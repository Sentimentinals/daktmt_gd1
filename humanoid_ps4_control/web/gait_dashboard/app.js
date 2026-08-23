import * as THREE from "/vendor/three.module.min.js";

const $ = (id) => document.getElementById(id);
const DEG = Math.PI / 180;
const liveLimit = 3600;
const CONTROL_REQUEST_TIMEOUT_MS = 350;

let model = null;
let sceneState = null;
let liveFrames = [];
let replayFrames = [];
let liveMode = true;
let replayPlaying = false;
let replayIndex = 0;
let lastEventAt = 0;
let lastChartAt = 0;
let activeSection = "control";

const control = {
  clientId: globalThis.crypto?.randomUUID?.() || `browser-${Date.now()}-${Math.random()}`,
  sequence: 0,
  armed: false,
  mode: "manual",
  axes: { forward: 0, turn: 0, side: 0 },
  held: new Set(),
  actions: new Set(),
  sending: false,
};

function releaseMotion() {
  control.axes.forward = 0;
  control.axes.turn = 0;
  control.axes.side = 0;
  control.held.clear();
  document.querySelectorAll("[data-axis].active, [data-hold].active, [data-action].active").forEach((button) => {
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

function updateControlUI(state = {}) {
  if (typeof state.armed === "boolean") control.armed = state.armed;
  if (state.mode) control.mode = state.mode;
  $("armButton").classList.toggle("active", control.armed);
  $("armButton").textContent = control.armed ? "Disable control" : "Enable control";
  $("controlStatus").textContent = control.armed ? "ARMED" : "Disabled";
  $("controlStatus").style.color = control.armed ? "#45d09a" : "#aeb7bf";
  document.querySelectorAll("[data-mode]").forEach((button) => {
    button.classList.toggle("active", button.dataset.mode === control.mode);
  });
  document.querySelectorAll("[data-mode-actions]").forEach((group) => {
    group.classList.toggle("active", group.dataset.modeActions === control.mode);
  });
  document.querySelectorAll("[data-axis], [data-action], [data-hold]").forEach((button) => {
    const manualOnly = button.closest(".action-grid") || button.hasAttribute("data-axis");
    const modeGroup = button.closest("[data-mode-actions]");
    const modeAllowed = manualOnly ? control.mode === "manual" : !modeGroup || modeGroup.dataset.modeActions === control.mode;
    button.disabled = !control.armed || !modeAllowed;
  });
}

async function sendControl(emergencyStop = false) {
  if (control.sending && !emergencyStop) return;
  const sentActions = [...control.actions];
  const payload = {
    client_id: control.clientId,
    sequence: ++control.sequence,
    armed: control.armed,
    mode: control.mode,
    axes: control.axes,
    held: [...control.held],
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
    if (!response.ok) {
      const message = state.error || `Control HTTP ${response.status}`;
      if (response.status === 400 || response.status === 409) {
        releaseMotion();
        control.armed = false;
        updateControlUI({ runtime_status: message });
      }
      console.warn(message);
      return;
    }
    sentActions.forEach((action) => control.actions.delete(action));
    updateControlUI(state);
  } catch (error) {
    if (error.name !== "AbortError") console.warn("Control heartbeat failed", error);
  } finally {
    clearTimeout(requestTimeout);
    control.sending = false;
  }
}

function queueAction(action) {
  const available = [...document.querySelectorAll(`[data-action="${action}"]`)]
    .some((button) => !button.disabled);
  if (!control.armed || !available) return;
  control.actions.add(action);
  sendControl();
}

function setAxis(name, value, button) {
  if (!control.armed || control.mode !== "manual") return;
  control.axes[name] = Number(value);
  setButtonActive(button, Number(value) !== 0);
  sendControl();
}

function setHeld(name, enabled, button) {
  if (!control.armed) return;
  if (enabled) control.held.add(name);
  else control.held.delete(name);
  setButtonActive(button, enabled);
  sendControl();
}

function setCameraStreaming(enabled) {
  const feed = $("cameraFeed");
  if (enabled && !feed.getAttribute("src")) {
    feed.src = `/camera.mjpg?t=${Date.now()}`;
  } else if (!enabled) {
    feed.removeAttribute("src");
  }
}

function fixed(value, digits = 1, suffix = "") {
  return Number.isFinite(Number(value)) ? `${Number(value).toFixed(digits)}${suffix}` : "--";
}

function vectorPair(value) {
  return Array.isArray(value) ? `${fixed(value[0])} / ${fixed(value[1])} mm` : "--";
}

function posePwm(frame, servoId) {
  const pose = frame?.pose_pwm || {};
  const standing = model?.standing_pwm || {};
  return Number(pose[String(servoId)] ?? standing[String(servoId)] ?? 1500);
}

function debugPwm(frame, servoId) {
  const value = posePwm(frame, servoId);
  const base = Number(model?.standing_pwm?.[String(servoId)] ?? 1500);
  const delta = Math.round(value - base);
  return `${Math.round(value)} (${delta >= 0 ? "+" : ""}${delta})`;
}

function setMeter(element, value, limit = 12) {
  const normalized = Math.max(-1, Math.min(1, Number(value) / limit));
  element.style.left = normalized < 0 ? `${50 + normalized * 50}%` : "50%";
  element.style.width = `${Math.abs(normalized) * 50}%`;
  element.style.background = Math.abs(normalized) > 0.65 ? "#ee6b6e" : "#51b9d4";
}

function setForce(value, label, bar) {
  if (!Number.isFinite(Number(value))) {
    label.textContent = "--";
    bar.style.width = "0";
    return;
  }
  const force = Math.max(0, Math.min(1, Number(value)));
  label.textContent = `${Math.round(force * 100)}%`;
  bar.style.width = `${force * 100}%`;
  bar.style.background = force > 0.08 ? "#45d09a" : "#69737d";
}

function updateReadouts(frame) {
  if (!frame) return;
  const gait = frame.gait || {};
  const imu = frame.imu;
  const fsr = frame.fsr;
  const feet = gait.feet_mm || {};
  const commands = gait.commands || {};
  const phase = String(gait.phase || "idle").toUpperCase();
  const support = String(gait.support_leg || "double").toUpperCase();
  const swing = String(gait.swing_leg || "none").toUpperCase();

  $("modeBadge").textContent = frame.active ? "ACTIVE" : "IDLE";
  $("phaseName").textContent = phase;
  $("supportBadge").textContent = support;
  $("motionStatus").textContent = frame.status || "Waiting";
  $("balanceStatus").textContent = frame.balance_status || "--";
  $("stepCount").textContent = gait.step_count ?? 0;
  $("swingLeg").textContent = swing;

  $("rollValue").textContent = imu ? fixed(imu.roll_deg, 2, " deg") : "NO DATA";
  $("pitchValue").textContent = imu ? fixed(imu.pitch_deg, 2, " deg") : "NO DATA";
  $("yawValue").textContent = imu ? fixed(imu.yaw_deg, 2, " deg") : "NO DATA";
  setMeter($("rollMeter"), imu?.roll_deg ?? 0);
  setMeter($("pitchMeter"), imu?.pitch_deg ?? 0);

  setForce(fsr?.left, $("leftForce"), $("leftForceBar"));
  setForce(fsr?.right, $("rightForce"), $("rightForceBar"));
  $("comValue").textContent = vectorPair(gait.com_mm);
  $("zmpValue").textContent = vectorPair(gait.zmp_mm);
  $("leftFootZ").textContent = feet.left ? fixed(feet.left[2], 1, " mm") : "--";
  $("rightFootZ").textContent = feet.right ? fixed(feet.right[2], 1, " mm") : "--";
  $("forwardCommand").textContent = fixed(commands.forward_mm);
  $("turnCommand").textContent = fixed(commands.turn_mm);
  $("sideCommand").textContent = fixed(commands.side_mm);

  $("debugLift").textContent = fixed(gait.lift_factor, 2);
  $("debugLanding").textContent = fixed(gait.landing_progress, 2);
  $("debugCrouch").textContent = fixed(gait.crouch_mm, 1, " mm");
  const supportAnkle = support === "LEFT" ? 16 : support === "RIGHT" ? 17 : null;
  const swingAnkle = swing === "LEFT" ? 16 : swing === "RIGHT" ? 17 : null;
  $("debugSupportAnkle").textContent = supportAnkle ? `S${supportAnkle} ${debugPwm(frame, supportAnkle)}` : "--";
  $("debugSwingAnkle").textContent = swingAnkle ? `S${swingAnkle} ${debugPwm(frame, swingAnkle)}` : "--";
  [12, 13, 14, 15, 16].forEach((servoId) => {
    $(`debugL${servoId}`).textContent = debugPwm(frame, servoId);
  });
  [17, 18, 19, 20, 21].forEach((servoId) => {
    $(`debugR${servoId}`).textContent = debugPwm(frame, servoId);
  });

  const cameraOn = Boolean(frame.camera_ready);
  $("cameraBadge").textContent = cameraOn ? "LIVE" : "OFF";
  $("cameraBadge").classList.toggle("muted", !cameraOn);
  $("cameraFeed").style.display = cameraOn ? "block" : "none";
  $("cameraOffline").classList.toggle("hidden", cameraOn);
}

function makeRobot(robotModel, scene) {
  const dimensions = robotModel.dimensions_mm;
  const upper = dimensions.upper_leg;
  const lower = dimensions.lower_leg;
  const hipHalf = dimensions.half_hip;
  const hipHeight = upper + lower - 2;
  const frameMaterial = new THREE.MeshStandardMaterial({ color: 0x23282d, metalness: 0.78, roughness: 0.34 });
  const edgeMaterial = new THREE.MeshStandardMaterial({ color: 0x3b4249, metalness: 0.72, roughness: 0.30 });
  const servoMaterial = new THREE.MeshStandardMaterial({ color: 0x30363b, metalness: 0.48, roughness: 0.42 });
  const metalMaterial = new THREE.MeshStandardMaterial({ color: 0xb8c0c5, metalness: 0.92, roughness: 0.22 });
  const holeMaterial = new THREE.MeshStandardMaterial({ color: 0x080a0c, metalness: 0.15, roughness: 0.58 });
  const orangeMaterial = new THREE.MeshStandardMaterial({ color: 0xf06a22, emissive: 0x4a1605, roughness: 0.54 });
  const redMaterial = new THREE.MeshStandardMaterial({ color: 0xc53b32, emissive: 0x310706, roughness: 0.58 });
  const leftMaterial = new THREE.MeshBasicMaterial({ color: 0x45d09a });
  const rightMaterial = new THREE.MeshBasicMaterial({ color: 0xf4b84a });
  const root = new THREE.Group();
  scene.add(root);

  function addBox(parent, size, position, material) {
    const mesh = new THREE.Mesh(new THREE.BoxGeometry(...size), material);
    mesh.position.set(...position);
    parent.add(mesh);
    return mesh;
  }

  function addJoint(parent, radius = 8, z = 17) {
    const axle = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, 5, 20), metalMaterial);
    axle.rotation.x = Math.PI / 2;
    axle.position.z = z;
    parent.add(axle);
    const center = new THREE.Mesh(new THREE.CylinderGeometry(radius * 0.38, radius * 0.38, 5.6, 16), holeMaterial);
    center.rotation.x = Math.PI / 2;
    center.position.z = z + 0.4;
    parent.add(center);
  }

  function addScrews(parent, width, height, z, centerY = 0) {
    for (const x of [-width * 0.38, width * 0.38]) {
      for (const y of [-height * 0.38, height * 0.38]) {
        const screw = new THREE.Mesh(new THREE.CylinderGeometry(1.7, 1.7, 1.4, 12), metalMaterial);
        screw.rotation.x = Math.PI / 2;
        screw.position.set(x, centerY + y, z);
        parent.add(screw);
      }
    }
  }

  function addCable(parent, points, material = orangeMaterial) {
    const curve = new THREE.CatmullRomCurve3(points.map((point) => new THREE.Vector3(...point)));
    parent.add(new THREE.Mesh(new THREE.TubeGeometry(curve, 20, 1.15, 6, false), material));
  }

  function addServo(parent, size = [25, 38, 28], z = 0) {
    addBox(parent, size, [0, 0, z], servoMaterial);
    addBox(parent, [size[0] - 5, size[1] - 5, 2], [0, 0, z + size[2] / 2 + 1], edgeMaterial);
    addScrews(parent, size[0] - 5, size[1] - 5, z + size[2] / 2 + 2.2);
    addJoint(parent, Math.min(size[0], size[1]) * 0.25, z + size[2] / 2 + 4);
  }

  const pelvis = new THREE.Group();
  pelvis.position.y = hipHeight + 9;
  root.add(pelvis);
  addBox(pelvis, [92, 25, 24], [0, 0, 0], frameMaterial);
  addBox(pelvis, [62, 8, 28], [0, 0, 0], edgeMaterial);
  addScrews(pelvis, 82, 17, 13);

  const torsoPivot = new THREE.Group();
  torsoPivot.position.y = hipHeight + 20;
  root.add(torsoPivot);
  addBox(torsoPivot, [118, 24, 22], [0, 16, 0], frameMaterial);
  addBox(torsoPivot, [28, 72, 22], [0, 52, 0], frameMaterial);
  addBox(torsoPivot, [132, 18, 20], [0, 82, 0], edgeMaterial);
  addScrews(torsoPivot, 108, 16, 12, 16);
  for (const x of [-45, 45]) {
    const chestServo = new THREE.Group();
    chestServo.position.set(x, 56, 0);
    torsoPivot.add(chestServo);
    addServo(chestServo, [30, 44, 28]);
  }
  addCable(torsoPivot, [[-52, 70, 15], [-20, 84, 17], [20, 84, 17], [52, 70, 15]]);
  addCable(torsoPivot, [[-9, 18, 14], [-13, 48, 16], [-8, 80, 14]], redMaterial);

  const headPivot = new THREE.Group();
  headPivot.position.y = 102;
  torsoPivot.add(headPivot);
  addBox(headPivot, [18, 15, 22], [0, 0, 0], edgeMaterial);
  const headBody = new THREE.Group();
  headBody.position.y = 23;
  headPivot.add(headBody);
  addServo(headBody, [29, 42, 30], 0);
  const cameraEye = new THREE.Mesh(
    new THREE.CylinderGeometry(5.5, 5.5, 4, 18),
    new THREE.MeshStandardMaterial({ color: 0x111820, emissive: 0x123647, metalness: 0.6 }),
  );
  cameraEye.rotation.x = Math.PI / 2;
  cameraEye.position.set(0, 2, 20);
  headBody.add(cameraEye);

  function buildLeg(side, material) {
    const hipRoll = new THREE.Group();
    hipRoll.position.set(side * hipHalf, hipHeight, 0);
    root.add(hipRoll);
    addServo(hipRoll, [28, 36, 28]);
    const hipPitch = new THREE.Group();
    hipRoll.add(hipPitch);
    addBox(hipPitch, [26, upper - 17, 16], [0, -upper / 2, 0], frameMaterial);
    addBox(hipPitch, [5, upper - 13, 22], [side * 12, -upper / 2, 0], edgeMaterial);
    addScrews(hipPitch, 20, upper - 28, 9, -upper / 2);
    addCable(hipPitch, [[side * 15, -7, 15], [side * 19, -upper * 0.48, 17], [side * 14, -upper + 4, 15]]);
    const knee = new THREE.Group();
    knee.position.y = -upper;
    hipPitch.add(knee);
    addServo(knee, [28, 38, 30]);
    addBox(knee, [6, lower - 18, 18], [-9, -lower / 2, 0], edgeMaterial);
    addBox(knee, [6, lower - 18, 18], [9, -lower / 2, 0], edgeMaterial);
    addBox(knee, [24, 12, 18], [0, -lower * 0.52, 0], frameMaterial);
    addCable(knee, [[side * 14, -8, 16], [side * 18, -lower * 0.50, 18], [side * 13, -lower + 5, 16]], redMaterial);
    const anklePitch = new THREE.Group();
    anklePitch.position.y = -lower;
    knee.add(anklePitch);
    addServo(anklePitch, [29, 35, 30]);
    const ankleRoll = new THREE.Group();
    anklePitch.add(ankleRoll);
    addJoint(ankleRoll, 8, 18);
    addBox(ankleRoll, [42, 10, 72], [0, -11, 17], frameMaterial);
    addBox(ankleRoll, [46, 4, 35], [0, -15, 37], edgeMaterial);
    addScrews(ankleRoll, 32, 7, 54, -11);
    addBox(knee, [2.5, lower - 28, 3], [side * 15, -lower / 2, 12], material);
    return { hipRoll, hipPitch, knee, anklePitch, ankleRoll, material };
  }

  function buildArm(side) {
    const shoulderSwing = new THREE.Group();
    shoulderSwing.position.set(side * 73, 80, 0);
    torsoPivot.add(shoulderSwing);
    addServo(shoulderSwing, [30, 40, 28]);
    const upperArm = new THREE.Group();
    shoulderSwing.add(upperArm);
    addBox(upperArm, [21, 52, 15], [0, -31, 0], frameMaterial);
    addBox(upperArm, [5, 56, 21], [side * 11, -31, 0], edgeMaterial);
    addScrews(upperArm, 15, 42, 9, -31);
    addCable(upperArm, [[side * 14, -6, 14], [side * 17, -30, 16], [side * 13, -58, 14]]);
    const elbow = new THREE.Group();
    elbow.position.y = -62;
    upperArm.add(elbow);
    addServo(elbow, [27, 37, 28]);
    addBox(elbow, [18, 48, 14], [0, -30, 0], frameMaterial);
    addBox(elbow, [5, 51, 20], [side * 10, -30, 0], edgeMaterial);
    addCable(elbow, [[side * 13, -6, 14], [side * 16, -28, 16], [side * 11, -54, 13]], redMaterial);
    addBox(elbow, [15, 24, 18], [0, -65, 2], edgeMaterial);
    addJoint(elbow, 6, 16);
    return { shoulderSwing, upperArm, elbow };
  }

  const leftLeg = buildLeg(-1, leftMaterial);
  const rightLeg = buildLeg(1, rightMaterial);
  const leftArm = buildArm(-1);
  const rightArm = buildArm(1);

  const comMarker = new THREE.Mesh(
    new THREE.SphereGeometry(7, 16, 12),
    new THREE.MeshBasicMaterial({ color: 0x51b9d4 }),
  );
  scene.add(comMarker);
  const zmpMarker = new THREE.Mesh(
    new THREE.CylinderGeometry(8, 8, 2, 18),
    new THREE.MeshBasicMaterial({ color: 0xee6b6e }),
  );
  zmpMarker.position.y = 1;
  scene.add(zmpMarker);

  const targetMaterialLeft = new THREE.MeshBasicMaterial({ color: 0x45d09a, wireframe: true });
  const targetMaterialRight = new THREE.MeshBasicMaterial({ color: 0xf4b84a, wireframe: true });
  const leftTarget = new THREE.Mesh(new THREE.BoxGeometry(38, 5, 66), targetMaterialLeft);
  const rightTarget = new THREE.Mesh(new THREE.BoxGeometry(38, 5, 66), targetMaterialRight);
  scene.add(leftTarget, rightTarget);

  const leftTrail = new THREE.Line(
    new THREE.BufferGeometry(),
    new THREE.LineBasicMaterial({ color: 0x45d09a }),
  );
  const rightTrail = new THREE.Line(
    new THREE.BufferGeometry(),
    new THREE.LineBasicMaterial({ color: 0xf4b84a }),
  );
  scene.add(leftTrail, rightTrail);

  return {
    root,
    torsoPivot,
    headPivot,
    leftLeg,
    rightLeg,
    leftArm,
    rightArm,
    comMarker,
    zmpMarker,
    leftTarget,
    rightTarget,
    leftTrail,
    rightTrail,
    rotations: [],
  };
}

function initScene(robotModel) {
  const container = $("robotViewport");
  try {
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0b0e11);
    scene.fog = new THREE.Fog(0x0b0e11, 440, 820);
    const camera = new THREE.PerspectiveCamera(42, 1, 1, 1500);
    camera.position.set(0, 200, 560);
    const renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: "high-performance" });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 1.5));
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    container.prepend(renderer.domElement);

    scene.add(new THREE.HemisphereLight(0xe8f0f4, 0x20252a, 2.1));
    scene.add(new THREE.AmbientLight(0xffffff, 0.58));
    const keyLight = new THREE.DirectionalLight(0xffffff, 2.3);
    keyLight.position.set(170, 300, 220);
    scene.add(keyLight);
    const fillLight = new THREE.DirectionalLight(0x91b7cf, 1.35);
    fillLight.position.set(-190, 190, 260);
    scene.add(fillLight);
    const rimLight = new THREE.DirectionalLight(0xb8d5e4, 1.05);
    rimLight.position.set(230, 170, -140);
    scene.add(rimLight);
    const grid = new THREE.GridHelper(520, 26, 0x3d474f, 0x232a30);
    scene.add(grid);

    const robot = makeRobot(robotModel, scene);
    const cameraTarget = new THREE.Vector3(0, 145, 0);
    let desiredCamera = new THREE.Vector3(0, 200, 560);
    let orbitTheta = 0;
    let orbitPhi = 1.46;
    let orbitRadius = 560;
    let dragging = false;
    let pointerX = 0;
    let pointerY = 0;

    function orbitPosition() {
      return new THREE.Vector3(
        orbitRadius * Math.sin(orbitPhi) * Math.sin(orbitTheta),
        cameraTarget.y + orbitRadius * Math.cos(orbitPhi),
        orbitRadius * Math.sin(orbitPhi) * Math.cos(orbitTheta),
      );
    }

    function setView(view) {
      if (view === "front" || view === "fit") {
        orbitTheta = 0;
        orbitPhi = 1.46;
        orbitRadius = 560;
      } else if (view === "side") {
        orbitTheta = Math.PI / 2;
        orbitPhi = 1.46;
        orbitRadius = 560;
      } else if (view === "top") {
        orbitTheta = 0;
        orbitPhi = 0.12;
        orbitRadius = 520;
      }
      desiredCamera = orbitPosition();
    }

    container.addEventListener("pointerdown", (event) => {
      dragging = true;
      pointerX = event.clientX;
      pointerY = event.clientY;
      container.setPointerCapture(event.pointerId);
    });
    container.addEventListener("pointermove", (event) => {
      if (!dragging) return;
      orbitTheta -= (event.clientX - pointerX) * 0.008;
      orbitPhi = Math.max(0.12, Math.min(Math.PI - 0.12, orbitPhi + (event.clientY - pointerY) * 0.008));
      pointerX = event.clientX;
      pointerY = event.clientY;
      desiredCamera = orbitPosition();
    });
    container.addEventListener("pointerup", () => { dragging = false; });
    container.addEventListener("wheel", (event) => {
      event.preventDefault();
      orbitRadius = Math.max(230, Math.min(700, orbitRadius + event.deltaY * 0.35));
      desiredCamera = orbitPosition();
    }, { passive: false });
    document.querySelectorAll("[data-view]").forEach((button) => {
      button.addEventListener("click", () => setView(button.dataset.view));
    });

    function resize() {
      const width = Math.max(1, container.clientWidth);
      const height = Math.max(1, container.clientHeight);
      renderer.setSize(width, height, false);
      camera.aspect = width / height;
      camera.updateProjectionMatrix();
    }
    new ResizeObserver(resize).observe(container);
    resize();

    function animate() {
      camera.position.lerp(desiredCamera, 0.11);
      camera.lookAt(cameraTarget);
      robot.rotations.forEach((entry) => {
        entry.object.rotation[entry.axis] += (entry.target - entry.object.rotation[entry.axis]) * 0.24;
      });
      renderer.render(scene, camera);
      requestAnimationFrame(animate);
    }
    requestAnimationFrame(animate);
    sceneState = { scene, camera, renderer, robot, setView };
  } catch (error) {
    console.error(error);
    $("webglFallback").classList.remove("hidden");
  }
}

function jointAngle(frame, servoId) {
  const pose = frame?.pose_pwm || {};
  const standing = model?.standing_pwm || {};
  const baseAngles = model?.base_angles_deg || {};
  const directions = model?.directions || {};
  const pwm = Number(pose[String(servoId)] ?? standing[String(servoId)] ?? 1500);
  const basePwm = Number(standing[String(servoId)] ?? 1500);
  const baseAngle = Number(baseAngles[String(servoId)] ?? 0);
  const direction = Number(directions[String(servoId)] ?? 1);
  return baseAngle + (pwm - basePwm) / (direction * Number(model?.pwm_per_deg || 11.111));
}

function pwmDeltaDeg(frame, servoId) {
  const pose = frame?.pose_pwm || {};
  const standing = model?.standing_pwm || {};
  return (Number(pose[String(servoId)] ?? 1500) - Number(standing[String(servoId)] ?? 1500)) /
    Number(model?.pwm_per_deg || 11.111);
}

function setRotation(robot, object, axis, target) {
  let entry = robot.rotations.find((item) => item.object === object && item.axis === axis);
  if (!entry) {
    entry = { object, axis, target };
    robot.rotations.push(entry);
  }
  entry.target = target;
}

function engineToWorld(point) {
  if (!Array.isArray(point)) return new THREE.Vector3();
  return new THREE.Vector3(Number(point[1]), Number(point[2]), Number(point[0]));
}

function updateTrail(line, frames, endIndex, foot) {
  const points = [];
  const start = Math.max(0, endIndex - 100);
  for (let index = start; index <= endIndex; index += 2) {
    const point = frames[index]?.gait?.feet_mm?.[foot];
    if (point) {
      const world = engineToWorld(point);
      world.y += 3;
      points.push(world);
    }
  }
  line.geometry.dispose();
  line.geometry = new THREE.BufferGeometry().setFromPoints(points);
}

function updateScene(frame, frames, index) {
  if (!sceneState || !frame || !model) return;
  const robot = sceneState.robot;
  const left = robot.leftLeg;
  const right = robot.rightLeg;
  setRotation(robot, left.hipRoll, "z", -jointAngle(frame, 12) * DEG);
  setRotation(robot, left.hipPitch, "x", jointAngle(frame, 13) * DEG);
  setRotation(robot, left.knee, "x", -jointAngle(frame, 14) * DEG);
  setRotation(robot, left.anklePitch, "x", jointAngle(frame, 15) * DEG);
  setRotation(robot, left.ankleRoll, "z", -jointAngle(frame, 16) * DEG);
  setRotation(robot, right.hipRoll, "z", jointAngle(frame, 21) * DEG);
  setRotation(robot, right.hipPitch, "x", jointAngle(frame, 20) * DEG);
  setRotation(robot, right.knee, "x", -jointAngle(frame, 19) * DEG);
  setRotation(robot, right.anklePitch, "x", jointAngle(frame, 18) * DEG);
  setRotation(robot, right.ankleRoll, "z", jointAngle(frame, 17) * DEG);

  setRotation(robot, robot.leftArm.shoulderSwing, "x", pwmDeltaDeg(frame, 11) * DEG);
  setRotation(robot, robot.leftArm.upperArm, "z", pwmDeltaDeg(frame, 10) * DEG);
  setRotation(robot, robot.leftArm.elbow, "x", pwmDeltaDeg(frame, 9) * DEG);
  setRotation(robot, robot.rightArm.shoulderSwing, "x", -pwmDeltaDeg(frame, 22) * DEG);
  setRotation(robot, robot.rightArm.upperArm, "z", -pwmDeltaDeg(frame, 23) * DEG);
  setRotation(robot, robot.rightArm.elbow, "x", -pwmDeltaDeg(frame, 24) * DEG);
  setRotation(robot, robot.headPivot, "y", pwmDeltaDeg(frame, 25) * DEG);

  const imu = frame.imu;
  setRotation(robot, robot.torsoPivot, "z", imu ? -Number(imu.roll_deg) * DEG : 0);
  setRotation(robot, robot.torsoPivot, "x", imu ? Number(imu.pitch_deg) * DEG : 0);

  const gait = frame.gait || {};
  const feet = gait.feet_mm || {};
  const com = engineToWorld(gait.com_mm);
  const zmp = engineToWorld(gait.zmp_mm);
  robot.comMarker.position.copy(com);
  robot.zmpMarker.position.set(zmp.x, 1, zmp.z);
  if (feet.left) robot.leftTarget.position.copy(engineToWorld(feet.left)).add(new THREE.Vector3(0, 2, 0));
  if (feet.right) robot.rightTarget.position.copy(engineToWorld(feet.right)).add(new THREE.Vector3(0, 2, 0));

  const support = gait.support_leg;
  const swing = gait.swing_leg;
  const neutralColor = 0x7f8b94;
  robot.leftLeg.material.color.setHex(support === "left" ? 0x45d09a : swing === "left" ? 0xf4b84a : neutralColor);
  robot.rightLeg.material.color.setHex(support === "right" ? 0x45d09a : swing === "right" ? 0xf4b84a : neutralColor);
  updateTrail(robot.leftTrail, frames, index, "left");
  updateTrail(robot.rightTrail, frames, index, "right");
}

function drawChart(canvas, frames, definitions, range) {
  const bounds = canvas.getBoundingClientRect();
  const ratio = Math.min(window.devicePixelRatio || 1, 1.5);
  const width = Math.max(1, Math.round(bounds.width * ratio));
  const height = Math.max(1, Math.round(bounds.height * ratio));
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  const context = canvas.getContext("2d");
  context.clearRect(0, 0, width, height);
  context.strokeStyle = "#273039";
  context.lineWidth = 1;
  for (let row = 1; row < 4; row++) {
    const y = row * height / 4;
    context.beginPath();
    context.moveTo(0, y);
    context.lineTo(width, y);
    context.stroke();
  }
  if (frames.length < 2) return;
  const min = range[0];
  const max = range[1];
  definitions.forEach((definition) => {
    context.strokeStyle = definition.color;
    context.lineWidth = 1.7 * ratio;
    context.beginPath();
    let started = false;
    frames.forEach((frame, index) => {
      const value = definition.value(frame);
      if (!Number.isFinite(Number(value))) {
        started = false;
        return;
      }
      const x = index * width / Math.max(1, frames.length - 1);
      const y = height - (Math.max(min, Math.min(max, Number(value))) - min) * height / (max - min);
      if (!started) {
        context.moveTo(x, y);
        started = true;
      } else {
        context.lineTo(x, y);
      }
    });
    context.stroke();
  });
}

function refreshCharts(frames, index) {
  const start = Math.max(0, index - 220);
  const visible = frames.slice(start, index + 1);
  drawChart($("imuChart"), visible, [
    { color: "#51b9d4", value: (frame) => frame.imu?.roll_deg },
    { color: "#ee6b6e", value: (frame) => frame.imu?.pitch_deg },
  ], [-15, 15]);
  drawChart($("footChart"), visible, [
    { color: "#45d09a", value: (frame) => frame.gait?.feet_mm?.left?.[2] },
    { color: "#f4b84a", value: (frame) => frame.gait?.feet_mm?.right?.[2] },
  ], [0, 70]);
}

function showFrame(frame, frames, index) {
  if (!frame) return;
  updateReadouts(frame);
  updateScene(frame, frames, index);
  const now = performance.now();
  if (now - lastChartAt > 100) {
    refreshCharts(frames, index);
    lastChartAt = now;
  }
  $("replaySlider").max = Math.max(0, frames.length - 1);
  $("replaySlider").value = index;
  $("frameCounter").textContent = `${frames.length} frames`;
  $("replayTime").textContent = liveMode ? "LIVE" : fixed(frame.time_s, 2, " s");
}

function setLive() {
  liveMode = true;
  replayPlaying = false;
  $("liveButton").classList.add("active");
  $("playButton").textContent = "Pause";
  if (liveFrames.length) showFrame(liveFrames.at(-1), liveFrames, liveFrames.length - 1);
}

function pauseLive() {
  replayFrames = [...liveFrames];
  liveMode = false;
  replayPlaying = false;
  replayIndex = Math.max(0, replayFrames.length - 1);
  $("liveButton").classList.remove("active");
  $("playButton").textContent = "Play";
  showFrame(replayFrames[replayIndex], replayFrames, replayIndex);
}

function replayTick() {
  if (!liveMode && replayPlaying && replayFrames.length) {
    replayIndex += 1;
    if (replayIndex >= replayFrames.length) {
      replayIndex = replayFrames.length - 1;
      replayPlaying = false;
      $("playButton").textContent = "Play";
    }
    showFrame(replayFrames[replayIndex], replayFrames, replayIndex);
  }
  setTimeout(replayTick, 80);
}

async function loadSessions() {
  try {
    const sessions = await fetch("/api/sessions", { cache: "no-store" }).then((response) => response.json());
    const select = $("sessionSelect");
    const selected = select.value;
    select.innerHTML = '<option value="">Recorded sessions</option>';
    sessions.forEach((session) => {
      const option = document.createElement("option");
      option.value = session.name;
      option.textContent = `${session.modified.replace("T", " ")}  ${Math.round(session.size / 1024)} KB`;
      select.appendChild(option);
    });
    if ([...select.options].some((option) => option.value === selected)) select.value = selected;
  } catch (error) {
    console.warn("Cannot load sessions", error);
  }
}

async function loadSelectedSession() {
  const name = $("sessionSelect").value;
  if (!name) return;
  const text = await fetch(`/api/session?name=${encodeURIComponent(name)}`, { cache: "no-store" })
    .then((response) => {
      if (!response.ok) throw new Error(`Session HTTP ${response.status}`);
      return response.text();
    });
  replayFrames = text.split(/\r?\n/).filter(Boolean).map((line) => JSON.parse(line));
  liveMode = false;
  replayPlaying = false;
  replayIndex = 0;
  $("liveButton").classList.remove("active");
  $("playButton").textContent = "Play";
  showFrame(replayFrames[0], replayFrames, 0);
}

function exportFrames() {
  const frames = liveMode ? liveFrames : replayFrames;
  if (!frames.length) return;
  const payload = frames.map((frame) => JSON.stringify(frame)).join("\n") + "\n";
  const link = document.createElement("a");
  link.href = URL.createObjectURL(new Blob([payload], { type: "application/x-ndjson" }));
  link.download = `gait_export_${new Date().toISOString().replace(/[:.]/g, "-")}.jsonl`;
  link.click();
  setTimeout(() => URL.revokeObjectURL(link.href), 1000);
}

function connectStream() {
  const source = new EventSource("/api/events");
  source.onmessage = (event) => {
    try {
      const frame = JSON.parse(event.data);
      liveFrames.push(frame);
      if (liveFrames.length > liveLimit) liveFrames.splice(0, liveFrames.length - liveLimit);
      lastEventAt = performance.now();
      $("streamState").classList.remove("offline");
      $("streamLabel").textContent = "Connected";
      if (liveMode) showFrame(frame, liveFrames, liveFrames.length - 1);
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
  document.querySelectorAll("[data-section]").forEach((button) => {
    button.addEventListener("click", () => {
      activeSection = button.dataset.section;
      releaseMotion();
      sendControl();
      document.querySelectorAll("[data-section]").forEach((item) => {
        item.classList.toggle("active", item === button);
      });
      $("controlSection").classList.toggle("active", activeSection === "control");
      $("analysisSection").classList.toggle("active", activeSection === "analysis");
      setCameraStreaming(activeSection === "control");
    });
  });

  $("armButton").addEventListener("click", () => {
    releaseMotion();
    control.armed = !control.armed;
    updateControlUI();
    sendControl();
  });
  $("emergencyButton").addEventListener("click", () => {
    releaseMotion();
    control.armed = false;
    updateControlUI({ runtime_status: "Emergency stop requested" });
    sendControl(true);
  });

  document.querySelectorAll("[data-mode]").forEach((button) => {
    button.addEventListener("click", () => {
      releaseMotion();
      control.mode = button.dataset.mode;
      updateControlUI();
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
  document.querySelectorAll("[data-hold]").forEach((button) => {
    button.addEventListener("pointerdown", (event) => {
      event.preventDefault();
      button.setPointerCapture(event.pointerId);
      setHeld(button.dataset.hold, true, button);
    });
    button.addEventListener("pointerup", () => setHeld(button.dataset.hold, false, button));
    button.addEventListener("pointercancel", () => setHeld(button.dataset.hold, false, button));
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
    g: "getup_front",
    b: "getup_back",
    c: "reset",
    v: "terrain_toggle",
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
    } else if (key === "r" && control.mode === "pickup") {
      event.preventDefault();
      if (event.repeat) return;
      setHeld("squat", true, document.querySelector('[data-hold="squat"]'));
    } else if (actionKeys[key] && !event.repeat) {
      event.preventDefault();
      setActionActive(actionKeys[key], true);
      queueAction(actionKeys[key]);
    } else if (key === "Escape") {
      event.preventDefault();
      releaseMotion();
      setButtonActive($("emergencyButton"), true);
      control.armed = false;
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
    } else if (key === "r") {
      setHeld("squat", false, document.querySelector('[data-hold="squat"]'));
    } else if (actionKeys[key]) {
      setActionActive(actionKeys[key], false);
    } else if (key === "Escape") {
      setButtonActive($("emergencyButton"), false);
    }
  });
  window.addEventListener("blur", () => {
    releaseMotion();
    sendControl();
  });
  document.addEventListener("visibilitychange", () => {
    if (document.hidden) {
      releaseMotion();
      sendControl();
    }
  });
  window.addEventListener("beforeunload", () => {
    const payload = JSON.stringify({
      client_id: control.clientId,
      sequence: ++control.sequence,
      armed: false,
      mode: control.mode,
      axes: { forward: 0, turn: 0, side: 0 },
      held: [],
      actions: [],
    });
    navigator.sendBeacon("/api/control", payload);
  });
  $("cameraFeed").addEventListener("error", () => {
    $("cameraFeed").style.display = "none";
    $("cameraOffline").classList.remove("hidden");
  });
  updateControlUI();
  setInterval(() => sendControl(), 100);
}

function bindControls() {
  $("liveButton").addEventListener("click", setLive);
  $("playButton").addEventListener("click", () => {
    if (liveMode) {
      pauseLive();
      return;
    }
    replayPlaying = !replayPlaying;
    $("playButton").textContent = replayPlaying ? "Pause" : "Play";
  });
  $("replaySlider").addEventListener("input", (event) => {
    if (liveMode) pauseLive();
    replayPlaying = false;
    $("playButton").textContent = "Play";
    replayIndex = Number(event.target.value);
    showFrame(replayFrames[replayIndex], replayFrames, replayIndex);
  });
  $("loadSessionButton").addEventListener("click", () => loadSelectedSession().catch(console.error));
  $("exportButton").addEventListener("click", exportFrames);
}

async function start() {
  model = await fetch("/api/model", { cache: "no-store" }).then((response) => response.json());
  initScene(model);
  bindWebControl();
  if (location.hash === "#analysis") document.querySelector('[data-section="analysis"]').click();
  bindControls();
  try {
    liveFrames = await fetch("/api/history", { cache: "no-store" }).then((response) => response.json());
    if (liveFrames.length) showFrame(liveFrames.at(-1), liveFrames, liveFrames.length - 1);
  } catch (error) {
    console.warn("No initial history", error);
  }
  connectStream();
  loadSessions();
  replayTick();
  setInterval(loadSessions, 15000);
  setInterval(() => {
    $("clock").textContent = new Date().toLocaleTimeString("vi-VN", { hour12: false });
    if (lastEventAt && performance.now() - lastEventAt > 3000) {
      $("streamState").classList.add("offline");
      $("streamLabel").textContent = "Telemetry stale";
    }
  }, 1000);
}

start().catch((error) => {
  console.error(error);
  $("streamState").classList.add("offline");
  $("streamLabel").textContent = "Dashboard failed";
});
