const $ = (id) => document.getElementById(id);
const CONTROL_REQUEST_TIMEOUT_MS = 350;
const MODE_LABELS = {
  manual: "MANUAL",
  terrain: "TERRAIN AUTO",
  follow: "PERSON FOLLOW",
};

let lastEventAt = 0;
let cameraAvailable = false;
let cameraRetry = null;

function updateCamera(available, error) {
  cameraAvailable = available;
  const feed = $("cameraFeed");
  if (!available) {
    clearTimeout(cameraRetry);
    cameraRetry = null;
    feed.removeAttribute("src");
  } else if (!feed.hasAttribute("src") && cameraRetry === null) {
    feed.src = `/camera.mjpg?t=${Date.now()}`;
  }
  const connected = available && cameraRetry === null;
  $("cameraBadge").textContent = connected ? "LIVE" : available ? "RECONNECTING" : "OFF";
  $("cameraBadge").classList.toggle("muted", !connected);
  feed.style.display = connected ? "block" : "none";
  $("cameraOffline").classList.toggle("hidden", connected);
  $("cameraOffline").textContent = error || (available ? "RECONNECTING CAMERA" : "CAMERA OFFLINE");
}

const control = {
  clientId: globalThis.crypto?.randomUUID?.() || `browser-${Date.now()}-${Math.random()}`,
  sequence: 0,
  armed: false,
  mode: "manual",
  axes: { forward: 0, turn: 0, side: 0 },
  actions: new Set(),
  sending: false,
  pending: false,
};

function releaseMotion() {
  control.axes.forward = 0;
  control.axes.turn = 0;
  control.axes.side = 0;
  document.querySelectorAll("[data-axis].active, [data-action].active").forEach((button) => {
    button.classList.remove("active");
    button.setAttribute("aria-pressed", "false");
  });
  setButtonActive($("emergencyButton"), false);
}

function setButtonActive(button, enabled) {
  if (!button) return;
  button.classList.toggle("active", enabled);
  button.setAttribute("aria-pressed", String(enabled));
}

function setActionActive(action, enabled) {
  document.querySelectorAll(`[data-action="${action}"]`).forEach((button) => {
    if (!button.disabled || !enabled) setButtonActive(button, enabled);
  });
}

function showModeTransition() {
  const label = MODE_LABELS[control.mode] || control.mode.toUpperCase();
  $("motionStatus").textContent = control.armed ? `STARTING ${label}` : `${label} SELECTED`;
  $("motionStatus").title = $("motionStatus").textContent;
  $("balanceStatus").textContent = "--";
  $("balanceStatus").title = "--";
  $("stepCount").textContent = "0";
  $("swingLeg").textContent = "NONE";
  $("modeBadge").textContent = "IDLE";
}

function updateControlUI(state = {}) {
  if (typeof state.armed === "boolean") control.armed = state.armed;
  if (state.mode) control.mode = state.mode;
  $("armButton").classList.toggle("active", control.armed);
  $("armButton").textContent = control.armed ? "Disable control" : "Enable control";
  $("controlStatus").textContent = control.armed ? "ARMED" : "Disabled";
  $("controlStatus").classList.toggle("armed", control.armed);
  document.querySelectorAll("[data-mode]").forEach((button) => {
    const active = button.dataset.mode === control.mode;
    button.classList.toggle("active", active);
    button.setAttribute("aria-pressed", String(active));
  });
  document.querySelectorAll("[data-mode-actions]").forEach((group) => {
    group.classList.toggle("active", group.dataset.modeActions === control.mode);
  });
  document.querySelectorAll("[data-axis], [data-action]").forEach((button) => {
    const manualOnly = button.closest(".action-grid") || button.hasAttribute("data-axis");
    const modeGroup = button.closest("[data-mode-actions]");
    const modeAllowed = manualOnly ? control.mode === "manual" : !modeGroup || modeGroup.dataset.modeActions === control.mode;
    button.disabled = !control.armed || !modeAllowed;
  });
  if (state.runtime_status) {
    if (!state.runtime_mode || state.runtime_mode === control.mode) {
      $("motionStatus").textContent = state.runtime_status;
      $("motionStatus").title = state.runtime_status;
    } else {
      showModeTransition();
    }
  }
}

async function sendControl(emergencyStop = false, immediate = false) {
  const priority = emergencyStop || control.actions.has("reset") || control.actions.has("stop");
  if (control.sending && !priority && (!immediate || control.actions.size > 0)) {
    control.pending = true;
    return;
  }
  control.pending = false;
  const sentActions = [...control.actions];
  const payload = {
    client_id: control.clientId,
    sequence: ++control.sequence,
    armed: control.armed,
    mode: control.mode,
    axes: { ...control.axes },
    actions: sentActions,
    emergency_stop: emergencyStop,
  };
  const requestController = new AbortController();
  const requestTimeout = setTimeout(() => requestController.abort(), CONTROL_REQUEST_TIMEOUT_MS);
  control.sending = true;
  try {
    const response = await fetch("/api/control", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
      cache: "no-store",
      signal: requestController.signal,
    });
    const state = await response.json();
    if (payload.sequence !== control.sequence) return;
    if (!response.ok) {
      const message = state.error || `Control HTTP ${response.status}`;
      if (response.status === 400 || response.status === 409) {
        releaseMotion();
        control.armed = false;
        control.actions.clear();
        updateControlUI({ runtime_status: message });
      }
      console.warn(message);
      return;
    }
    sentActions.forEach((action) => control.actions.delete(action));
    // An older heartbeat must not undo a selection made while it was in flight.
    updateControlUI({
      ...state,
      mode: control.mode === payload.mode ? state.mode : control.mode,
      armed: control.armed === payload.armed ? state.armed : control.armed,
    });
  } catch (error) {
    if (error.name !== "AbortError") console.warn("Control heartbeat failed", error);
  } finally {
    clearTimeout(requestTimeout);
    if (payload.sequence === control.sequence) {
      control.sending = false;
      if (control.pending) sendControl();
    }
  }
}

function queueAction(action) {
  const available = [...document.querySelectorAll(`[data-action="${action}"]`)]
    .some((button) => !button.disabled);
  if (!control.armed || !available) return;
  if (action === "reset" || action === "stop") {
    releaseMotion();
    control.actions.clear();
    if (action === "reset") control.mode = "manual";
    updateControlUI();
  }
  control.actions.add(action);
  sendControl();
}

function setAxis(name, value, button) {
  if (!control.armed || control.mode !== "manual") return;
  control.axes[name] = Number(value);
  setButtonActive(button, Number(value) !== 0);
  sendControl(false, true);
}

function updateReadouts(frame) {
  const gait = frame.gait || {};
  const health = frame.gait_health || {};
  const maintenance = health.maintenance || {};
  if (!frame.runtime_mode || frame.runtime_mode === control.mode) {
    $("modeBadge").textContent = frame.active ? "ACTIVE" : "IDLE";
    $("motionStatus").textContent = frame.status || "Waiting";
    $("balanceStatus").textContent = frame.balance_status || "--";
    $("motionStatus").title = $("motionStatus").textContent;
    $("balanceStatus").title = $("balanceStatus").textContent;
    $("stepCount").textContent = gait.step_count ?? 0;
    $("swingLeg").textContent = String(gait.swing_leg || "none").toUpperCase();
  }
  const gaitHealth = $("gaitHealth");
  const maintenanceStatus = $("maintenanceStatus");
  gaitHealth.textContent = health.status || "OFF";
  maintenanceStatus.textContent = maintenance.state || "OFF";
  const score = Number(health.score);
  const threshold = Number(health.threshold);
  const hasScore = health.score !== null && health.score !== undefined
    && health.threshold !== null && health.threshold !== undefined
    && Number.isFinite(score) && Number.isFinite(threshold);
  gaitHealth.title = hasScore
    ? `${health.profile || "global"}: ${score.toFixed(2)} / ${threshold.toFixed(2)}`
    : health.model_error || "No trained gait baseline";
  const anomalyRate = Number(maintenance.anomaly_rate);
  const trend = Number(maintenance.trend_percent);
  maintenanceStatus.title = [
    maintenance.anomaly_rate !== null && maintenance.anomaly_rate !== undefined && Number.isFinite(anomalyRate)
      ? `Anomaly ${(anomalyRate * 100).toFixed(0)}%` : null,
    maintenance.trend_percent !== null && maintenance.trend_percent !== undefined && Number.isFinite(trend)
      ? `Trend ${trend >= 0 ? "+" : ""}${trend.toFixed(0)}%` : null,
  ].filter(Boolean).join(" | ") || "Session history not available";
  for (const element of [gaitHealth, maintenanceStatus]) {
    const label = element.textContent;
    element.classList.toggle("state-good", label === "NORMAL" || label === "GOOD");
    element.classList.toggle("state-watch", label === "SUSPECT" || label === "WATCH");
    element.classList.toggle("state-alert", label === "WARNING" || label === "CHECK ROBOT" || label === "MODEL ERROR");
  }
  updateCamera(Boolean(frame.camera_ready), frame.camera_error);
}

function connectStream() {
  const source = new EventSource("/api/events");
  source.onmessage = (event) => {
    try {
      const frame = JSON.parse(event.data);
      lastEventAt = performance.now();
      $("streamState").classList.remove("offline");
      $("streamLabel").textContent = "Connected";
      updateReadouts(frame);
    } catch (error) {
      console.warn("Invalid telemetry frame", error);
    }
  };
  source.onerror = () => {
    $("streamState").classList.add("offline");
    $("streamLabel").textContent = "Reconnecting";
  };
}

function bindWebControl() {
  $("armButton").addEventListener("click", () => {
    releaseMotion();
    control.armed = !control.armed;
    if (!control.armed) control.actions.clear();
    updateControlUI();
    showModeTransition();
    sendControl();
  });
  $("emergencyButton").addEventListener("click", () => {
    releaseMotion();
    control.armed = false;
    control.actions.clear();
    updateControlUI({ runtime_status: "Emergency stop requested" });
    sendControl(true);
  });

  document.querySelectorAll("[data-mode]").forEach((button) => {
    button.addEventListener("click", () => {
      releaseMotion();
      control.mode = button.dataset.mode;
      control.actions.clear();
      updateControlUI();
      showModeTransition();
      sendControl();
    });
  });
  document.querySelectorAll("[data-action]").forEach((button) => {
    button.addEventListener("click", () => queueAction(button.dataset.action));
  });
  document.querySelectorAll("[data-axis]").forEach((button) => {
    button.addEventListener("pointerdown", (event) => {
      event.preventDefault();
      button.setPointerCapture(event.pointerId);
      setAxis(button.dataset.axis, button.dataset.value, button);
    });
    button.addEventListener("pointerup", () => {
      setAxis(button.dataset.axis, 0, button);
    });
    button.addEventListener("pointercancel", () => {
      setAxis(button.dataset.axis, 0, button);
    });
  });
  const axisKeys = {
    ArrowUp: ["forward", 1],
    ArrowDown: ["forward", -1],
    ArrowLeft: ["turn", 1],
    ArrowRight: ["turn", -1],
    j: ["side", 1],
    k: ["side", -1],
  };
  const actionKeys = {
    " ": "stop",
    l: "dance",
    r: "squat",
    g: "getup_front",
    c: "reset",
    v: "terrain_toggle",
    u: "stair_toggle",
    y: "follow",
    n: "ignore_person",
  };
  window.addEventListener("keydown", (event) => {
    if (event.target.matches("input, select")) return;
    const key = event.key.length === 1 ? event.key.toLowerCase() : event.key;
    if (axisKeys[key]) {
      event.preventDefault();
      if (event.repeat) return;
      const [axis, value] = axisKeys[key];
      const button = document.querySelector(`[data-axis="${axis}"][data-value="${value}"]`);
      setAxis(axis, value, button);
    } else if (actionKeys[key] && !event.repeat) {
      event.preventDefault();
      setActionActive(actionKeys[key], true);
      queueAction(actionKeys[key]);
    } else if (key === "Escape") {
      event.preventDefault();
      releaseMotion();
      setButtonActive($("emergencyButton"), true);
      control.armed = false;
      control.actions.clear();
      updateControlUI({ runtime_status: "Emergency stop requested" });
      sendControl(true);
    }
  });
  window.addEventListener("keyup", (event) => {
    const key = event.key.length === 1 ? event.key.toLowerCase() : event.key;
    if (axisKeys[key]) {
      const [axis] = axisKeys[key];
      const button = document.querySelector(`[data-axis="${axis}"][data-value="${axisKeys[key][1]}"]`);
      setAxis(axis, 0, button);
    } else if (actionKeys[key]) {
      setActionActive(actionKeys[key], false);
    } else if (key === "Escape") {
      setButtonActive($("emergencyButton"), false);
    }
  });
  window.addEventListener("blur", () => {
    releaseMotion();
    sendControl(false, true);
  });
  document.addEventListener("visibilitychange", () => {
    if (document.hidden) {
      releaseMotion();
      sendControl(false, true);
    }
  });
  window.addEventListener("beforeunload", () => {
    const payload = JSON.stringify({
      client_id: control.clientId,
      sequence: ++control.sequence,
      armed: false,
      mode: control.mode,
      axes: { forward: 0, turn: 0, side: 0 },
      actions: [],
    });
    navigator.sendBeacon("/api/control", payload);
  });
  $("cameraFeed").addEventListener("error", () => {
    $("cameraFeed").removeAttribute("src");
    if (cameraAvailable && cameraRetry === null) {
      cameraRetry = setTimeout(() => {
        cameraRetry = null;
        updateCamera(cameraAvailable);
      }, 1000);
    }
    updateCamera(cameraAvailable);
  });
  updateControlUI();
  setInterval(() => sendControl(), 100);
}

function start() {
  bindWebControl();
  connectStream();
  setInterval(() => {
    $("clock").textContent = new Date().toLocaleTimeString("vi-VN", { hour12: false });
    if (lastEventAt && performance.now() - lastEventAt > 3000) {
      updateCamera(false);
      $("streamState").classList.add("offline");
      $("streamLabel").textContent = "Telemetry stale";
    }
  }, 1000);
}

start();
