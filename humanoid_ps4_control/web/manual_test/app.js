const state = {
  sequence: 0,
  renderedSequence: -1,
  armed: false,
  axes: { forward: 0, turn: 0, side: 0 },
};

const byId = (id) => document.getElementById(id);

function releaseMotion() {
  state.axes = { forward: 0, turn: 0, side: 0 };
  document.querySelectorAll("[data-axis].active").forEach((button) => button.classList.remove("active"));
}

function render(data) {
  if (Number(data.sequence) < state.renderedSequence) return;
  state.renderedSequence = Number(data.sequence);
  state.armed = Boolean(data.armed);
  byId("portValue").textContent = data.port || "--";
  byId("outputValue").textContent = state.armed ? "ENABLED" : "DISABLED";
  byId("outputValue").classList.toggle("live", state.armed);
  byId("commandValue").textContent = data.status || "IDLE";
  byId("phaseValue").textContent = String(data.gait?.phase || "idle").toUpperCase();
  byId("frameValue").textContent = String(data.frames ?? 0);
  byId("errorValue").textContent = data.error || "";
  byId("enableButton").textContent = state.armed ? "Disable output" : "Enable output";
  byId("enableButton").classList.toggle("active", state.armed);
  document.querySelectorAll("[data-axis], #stopButton").forEach((button) => {
    button.disabled = !state.armed;
  });
}

async function sendControl(extra = {}) {
  const payload = {
    sequence: ++state.sequence,
    armed: state.armed,
    axes: { ...state.axes },
    ...extra,
  };
  try {
    const response = await fetch("/api/control", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
      cache: "no-store",
    });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    render(await response.json());
  } catch (error) {
    byId("errorValue").textContent = `CONTROL LINK: ${error.message}`;
  }
}

function setAxis(axis, value, button) {
  if (!state.armed) return;
  state.axes[axis] = Number(value);
  button?.classList.toggle("active", Number(value) !== 0);
  sendControl();
}

byId("enableButton").addEventListener("click", () => {
  releaseMotion();
  state.armed = !state.armed;
  sendControl(state.armed ? {} : { stop: true });
});

byId("stopButton").addEventListener("click", () => {
  releaseMotion();
  byId("stopButton").classList.add("active");
  sendControl({ stop: true });
  setTimeout(() => byId("stopButton").classList.remove("active"), 160);
});

byId("resetButton").addEventListener("click", () => {
  releaseMotion();
  sendControl({ reset: true });
});

document.querySelectorAll("[data-axis]").forEach((button) => {
  const start = (event) => {
    event.preventDefault();
    setAxis(button.dataset.axis, button.dataset.value, button);
  };
  const stop = (event) => {
    event.preventDefault();
    setAxis(button.dataset.axis, 0, button);
  };
  button.addEventListener("pointerdown", start);
  button.addEventListener("pointerup", stop);
  button.addEventListener("pointercancel", stop);
  button.addEventListener("pointerleave", (event) => {
    if (button.classList.contains("active")) stop(event);
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

window.addEventListener("keydown", (event) => {
  const key = event.key.length === 1 ? event.key.toLowerCase() : event.key;
  if (axisKeys[key]) {
    event.preventDefault();
    if (event.repeat) return;
    const [axis, value] = axisKeys[key];
    const button = document.querySelector(`[data-axis="${axis}"][data-value="${value}"]`);
    setAxis(axis, value, button);
  } else if (key === " ") {
    event.preventDefault();
    releaseMotion();
    sendControl({ stop: true });
  } else if (key === "c") {
    event.preventDefault();
    releaseMotion();
    sendControl({ reset: true });
  }
});

window.addEventListener("keyup", (event) => {
  const key = event.key.length === 1 ? event.key.toLowerCase() : event.key;
  if (!axisKeys[key]) return;
  event.preventDefault();
  const [axis, value] = axisKeys[key];
  const button = document.querySelector(`[data-axis="${axis}"][data-value="${value}"]`);
  setAxis(axis, 0, button);
});

window.addEventListener("blur", () => {
  releaseMotion();
  if (state.armed) sendControl();
});

window.addEventListener("beforeunload", () => {
  navigator.sendBeacon("/api/control", JSON.stringify({
    sequence: ++state.sequence,
    armed: false,
    axes: { forward: 0, turn: 0, side: 0 },
    stop: true,
  }));
});

fetch("/api/state", { cache: "no-store" })
  .then((response) => response.json())
  .then(render)
  .catch((error) => { byId("errorValue").textContent = error.message; });

setInterval(() => {
  if (state.armed) sendControl();
}, 180);
