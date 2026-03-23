import * as THREE from 'three';
import { Reflector } from './utils/Reflector.js';
import { MuJoCoDemo } from './main.js';

export async function reloadFunc() {
  // Delete the old scene and load the new scene
  this.scene.remove(this.scene.getObjectByName("MuJoCo Root"));
  [this.model, this.data, this.bodies, this.lights] =
    await loadSceneFromURL(this.mujoco, this.params.scene, this);
  this.mujoco.mj_forward(this.model, this.data);
  for (let i = 0; i < this.updateGUICallbacks.length; i++) {
    this.updateGUICallbacks[i](this.model, this.data, this.params);
  }
}

/** @param {MuJoCoDemo} parentContext*/
export function setupGUI(parentContext) {

  // Reset camera on scene change/reload
  parentContext.updateGUICallbacks.length = 0;
  parentContext.updateGUICallbacks.push((model, data, params) => {
    parentContext.camera.position.set(2.0, 1.7, 1.7);
    parentContext.controls.target.set(0, 0.7, 0);
    parentContext.controls.update();
  });

  // Scene dropdown
  let reload = reloadFunc.bind(parentContext);
  parentContext.gui.add(parentContext.params, 'scene', {
    'Unitree G1': 'unitree_g1/scene.xml',
  }).name('Example Scene').onChange(reload);

  // ─── Simulation ──────────────────────────────────────────────────────────
  let simFolder = parentContext.gui.addFolder('⚙️ Simulation');

  const pauseSimulation = simFolder.add(parentContext.params, 'paused').name('Pause');
  pauseSimulation.onChange((value) => {
    const id = '__mujoco_paused_label';
    if (value) {
      if (!document.getElementById(id)) {
        const el = document.createElement('div');
        el.id = id;
        el.style.cssText = 'position:absolute;top:10px;left:10px;color:#fff;' +
          'font:bold 17px Arial;background:rgba(0,0,0,0.55);padding:4px 12px;' +
          'border-radius:6px;z-index:999;';
        el.textContent = '⏸  Paused';
        document.body.appendChild(el);
      }
    } else {
      const el = document.getElementById(id);
      if (el) el.remove();
    }
  });
  document.addEventListener('keydown', (e) => {
    if (e.code === 'Space') {
      parentContext.params.paused = !parentContext.params.paused;
      pauseSimulation.setValue(parentContext.params.paused);
      e.preventDefault();
    }
  });
  simFolder.add({ reload: () => reload() }, 'reload').name('Reload  [Ctrl+L]');
  document.addEventListener('keydown', (e) => {
    if (e.ctrlKey && e.code === 'KeyL') { reload(); e.preventDefault(); }
  });
  const resetSim = () => {
    parentContext.mujoco.mj_resetData(parentContext.model, parentContext.data);
    parentContext.mujoco.mj_forward(parentContext.model, parentContext.data);
  };
  simFolder.add({ reset: resetSim }, 'reset').name('Reset  [Backspace]');
  document.addEventListener('keydown', (e) => {
    if (e.code === 'Backspace') { resetSim(); e.preventDefault(); }
  });
  document.addEventListener('keydown', (e) => {
    if (e.ctrlKey && e.code === 'KeyA') {
      parentContext.camera.position.set(2.0, 1.7, 1.7);
      parentContext.controls.target.set(0, 0.7, 0);
      parentContext.controls.update();
      e.preventDefault();
    }
  });

  // ─── Joints grouped by limb ──────────────────────────────────────────────
  const LIMB_GROUPS = [
    { label: '🦵 Left Leg', prefixes: ['left_hip_', 'left_knee_', 'left_ankle_'] },
    { label: '🦵 Right Leg', prefixes: ['right_hip_', 'right_knee_', 'right_ankle_'] },
    { label: '🏋️ Waist / Back', prefixes: ['waist_'] },
    { label: '🤚 Left Arm', prefixes: ['left_shoulder_', 'left_elbow_', 'left_wrist_', 'left_hand_'] },
    { label: '🤚 Right Arm', prefixes: ['right_shoulder_', 'right_elbow_', 'right_wrist_', 'right_hand_'] },
  ];

  const textDecoder = new TextDecoder('utf-8');
  const nullChar = textDecoder.decode(new ArrayBuffer(1));

  let limbFolders = {};
  const actuatorFolder = parentContext.gui.addFolder('🦾 Joints');
  LIMB_GROUPS.forEach(g => {
    limbFolders[g.label] = actuatorFolder.addFolder(g.label);
    limbFolders[g.label].close();
  });
  const otherFolder = actuatorFolder.addFolder('⚙️ Other');
  otherFolder.close();
  actuatorFolder.close();

  const getActuatorName = (i) =>
    textDecoder.decode(
      parentContext.model.names.subarray(parentContext.model.name_actuatoradr[i])
    ).split(nullChar)[0];

  const getFolder = (name) => {
    for (const g of LIMB_GROUPS)
      for (const p of g.prefixes)
        if (name.startsWith(p)) return limbFolders[g.label];
    return otherFolder;
  };

  const addActuators = (model, data) => {
    const range = model.actuator_ctrlrange;
    const guis = [];
    for (let i = 0; i < model.nu; i++) {
      if (!model.actuator_ctrllimited[i]) continue;
      const name = getActuatorName(i);
      const label = name.replace(/_joint$/, '').replace(/_/g, ' ');
      parentContext.params[name] = 0.0;
      const g = getFolder(name)
        .add(parentContext.params, name, range[2 * i], range[2 * i + 1], 0.001)
        .name(label).listen();
      guis.push(g);
      g.onChange(v => {
        data.ctrl[i] = v;
        // If the simulation is paused, run physics substeps so that qpos
        // converges to the new ctrl target immediately (otherwise qpos stays
        // frozen and the robot won't visually move, and keyframe snapshots
        // would not reflect the desired pose).
        if (parentContext.params.paused) {
          for (let s = 0; s < 200; s++) {
            parentContext.mujoco.mj_step(parentContext.model, parentContext.data);
          }
        }
      });

    }
    return guis;
  };

  let actuatorGUIs = addActuators(parentContext.model, parentContext.data);
  parentContext.updateGUICallbacks.push((model, data) => {
    actuatorGUIs.forEach(g => g.destroy());
    actuatorGUIs = addActuators(model, data);
  });

  // ─── Keyframe Recorder ───────────────────────────────────────────────────
  // Workflow:
  //   1. Press "● Start Recording" → recording session opens (keyframe saving enabled)
  //   2. Adjust joints, press K or "Save Keyframe" to capture each pose
  //   3. Press "■ Stop Recording" → session ends, keyframes are kept
  //   4. Press "▶ Playback" to preview the animation directly in the browser
  //   5. Press "↓ Download Video" to render + export as .mp4 or .webm

  let savedKeyframes = [];
  let isRecSession = false;   // whether a recording session is active
  let hudEl = null;
  let hudTimer = null;
  let kfCountInput = null;
  let startRecBtn = null;    // dat.GUI controller refs for toggling names
  let stopRecBtn = null;

  // ── HUD helper ────────────────────────────────────────────────────────────
  const setHUD = (text, color) => {
    removeHUD();
    if (!text) return;
    hudEl = document.createElement('div');
    hudEl.id = '__mujoco_hud';
    hudEl.style.cssText =
      `position:absolute;top:12px;left:50%;transform:translateX(-50%);` +
      `background:${color};color:#fff;font:bold 14px "Courier New",monospace;` +
      `padding:5px 22px;border-radius:20px;z-index:1000;letter-spacing:2px;pointer-events:none;`;
    hudEl.textContent = text;
    document.body.appendChild(hudEl);
    // Blinking opacity
    let v = true;
    hudTimer = setInterval(() => {
      if (hudEl) hudEl.style.opacity = (v = !v) ? '1' : '0.2';
    }, 500);
  };
  const removeHUD = () => {
    if (hudTimer) { clearInterval(hudTimer); hudTimer = null; }
    if (hudEl) { hudEl.remove(); hudEl = null; }
  };

  // ── Count label refresh ───────────────────────────────────────────────────
  const updateKFCount = () => {
    const txt = isRecSession
      ? `● Recording  —  ${savedKeyframes.length} keyframe(s) saved`
      : `${savedKeyframes.length} keyframe(s) saved`;
    parentContext.params.__kfInfo = txt;
    if (kfCountInput) kfCountInput.value = txt;
  };

  // ── Build interpolated per-frame ctrl array ───────────────────────────────
  const buildFrames = (fps, secPerTransition) => {
    const fpt = Math.max(1, Math.round(fps * secPerTransition));
    const frames = [];
    for (let k = 0; k < savedKeyframes.length - 1; k++) {
      const A = savedKeyframes[k];
      const B = savedKeyframes[k + 1];
      for (let f = 0; f < fpt; f++) {
        const t = f / fpt;
        const ctrl = new Float32Array(A.length);
        for (let j = 0; j < A.length; j++) ctrl[j] = A[j] + (B[j] - A[j]) * t;
        frames.push(ctrl);
      }
    }
    // Hold final pose for 1 s
    const last = savedKeyframes[savedKeyframes.length - 1];
    for (let h = 0; h < fps; h++) frames.push(new Float32Array(last));
    return frames;
  };

  // Slider sync helper – called by main.js render loop during playback
  const sliderSync = (ctrl) => {
    for (let i = 0; i < parentContext.model.nu; i++) {
      if (!parentContext.model.actuator_ctrllimited[i]) continue;
      const nm = getActuatorName(i);
      if (nm in parentContext.params) parentContext.params[nm] = ctrl[i];
    }
  };

  // ── GUI folder ────────────────────────────────────────────────────────────
  const kfFolder = parentContext.gui.addFolder('🎬 Keyframe Recorder');

  // Read-only status label
  parentContext.params.__kfInfo = '0 keyframe(s) saved';
  const kfInfoCtrl = kfFolder.add(parentContext.params, '__kfInfo').name(' ');
  kfInfoCtrl.domElement.style.pointerEvents = 'none';
  kfCountInput = kfInfoCtrl.domElement.querySelector('input');
  if (kfCountInput) {
    kfCountInput.readOnly = true;
    kfCountInput.style.cssText =
      'background:transparent;border:none;color:#8cf;font-size:12px;cursor:default;width:100%;';
  }

  parentContext.params.kfDuration = 1.5;
  kfFolder.add(parentContext.params, 'kfDuration', 0.3, 8.0, 0.1).name('Transition (s)');

  parentContext.params.kfFPS = 30;
  kfFolder.add(parentContext.params, 'kfFPS', 15, 60, 1).name('FPS');

  const kfActions = {

    // ── Start recording session ──────────────────────────────────────────
    startRec: () => {
      if (isRecSession) return;
      savedKeyframes = [];
      isRecSession = true;
      setHUD('● REC', 'rgba(200,30,30,0.9)');
      updateKFCount();
    },

    // ── Stop recording session ───────────────────────────────────────────
    stopRec: () => {
      if (!isRecSession) return;
      isRecSession = false;
      removeHUD();
      updateKFCount();
    },

    // ── Save current pose as a keyframe ─────────────────────────────────
    saveKF: () => {
      if (!isRecSession) {
        if (kfCountInput) {
          const prev = kfCountInput.value;
          kfCountInput.style.color = '#fa0';
          kfCountInput.value = '⚠  Start recording first!';
          setTimeout(() => {
            if (kfCountInput) { kfCountInput.value = prev; kfCountInput.style.color = '#8cf'; }
          }, 1200);
        }
        return;
      }
      // Save qpos (actual joint angles) — this is what mj_forward uses to
      // compute body positions for rendering, not ctrl (which are target commands).
      savedKeyframes.push(new Float32Array(parentContext.data.qpos));
      console.log(`[KF] Saved keyframe ${savedKeyframes.length}, qpos[0..6]:`,
        Array.from(parentContext.data.qpos.slice(0, 7)).map(v => v.toFixed(3)));
      updateKFCount();
      if (kfCountInput) {
        kfCountInput.style.color = '#4f4';
        setTimeout(() => { if (kfCountInput) kfCountInput.style.color = '#8cf'; }, 350);
      }
    },


    // ── Remove last keyframe ─────────────────────────────────────────────
    undoKF: () => {
      if (savedKeyframes.length > 0) { savedKeyframes.pop(); updateKFCount(); }
    },

    // ── Playback animation in browser (RAF-driven) ───────────────────────
    playback: () => {
      if (savedKeyframes.length < 2) {
        alert('Save at least 2 keyframes first, then stop recording.');
        return;
      }
      if (isRecSession) {
        alert('Please stop the recording session first.');
        return;
      }
      if (parentContext._pbFrames) return; // already playing

      const fps = parentContext.params.kfFPS;
      const frames = buildFrames(fps, parentContext.params.kfDuration);
      const wasPaused = parentContext.params.paused;
      parentContext.params.paused = true;   // pause physics so render loop uses _pbFrames

      setHUD('▶ PLAYING', 'rgba(30,140,30,0.9)');

      // Hand frames to the render loop
      parentContext._pbFrames = frames;
      parentContext._pbIdx = 0;
      parentContext._pbSliderSync = sliderSync;
      parentContext._pbOnDone = () => {
        removeHUD();
        parentContext._pbSliderSync = null;
        parentContext.params.paused = wasPaused;
      };
    },

    // ── Download video via MediaRecorder (RAF-driven) ─────────────────────
    downloadVideo: () => {
      if (savedKeyframes.length < 2) {
        alert('Save at least 2 keyframes first, then stop recording.');
        return;
      }
      if (isRecSession) {
        alert('Please stop the recording session before downloading.');
        return;
      }
      if (parentContext._pbFrames) {
        alert('Playback is already running. Wait for it to finish first.');
        return;
      }

      const fps = parentContext.params.kfFPS;
      const frames = buildFrames(fps, parentContext.params.kfDuration);
      const wasPaused = parentContext.params.paused;
      parentContext.params.paused = true;

      // Start MediaRecorder BEFORE handing frames to the render loop
      const canvas = parentContext.renderer.domElement;
      const stream = canvas.captureStream(fps);
      let options;
      let ext = 'webm';

      if (MediaRecorder.isTypeSupported('video/mp4')) {
        options = { mimeType: 'video/mp4' };
        ext = 'mp4';
      } else if (MediaRecorder.isTypeSupported('video/webm;codecs=vp9')) {
        options = { mimeType: 'video/webm;codecs=vp9' };
      } else {
        options = { mimeType: 'video/webm' };
      }

      const mr = new MediaRecorder(stream, options);
      const chunks = [];
      mr.ondataavailable = (e) => { if (e.data && e.data.size > 0) chunks.push(e.data); };

      setHUD('▶ REC  EXPORTING…', 'rgba(160,30,160,0.9)');
      mr.start();

      // Hand frames to the render loop
      parentContext._pbFrames = frames;
      parentContext._pbIdx = 0;
      parentContext._pbSliderSync = sliderSync;
      parentContext._pbOnDone = () => {
        // Stop MediaRecorder – triggers saving the blob
        mr.onstop = () => {
          removeHUD();
          parentContext._pbSliderSync = null;
          const blob = new Blob(chunks, { type: options.mimeType });
          const url = URL.createObjectURL(blob);
          const a = document.createElement('a');
          const ts = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
          a.href = url;
          a.download = `mujoco_keyframes_${ts}.${ext}`;
          a.click();
          URL.revokeObjectURL(url);
          parentContext.params.paused = wasPaused;
        };
        mr.stop();
      };
    }
  };

  startRecBtn = kfFolder.add(kfActions, 'startRec').name('● Start Recording');
  stopRecBtn = kfFolder.add(kfActions, 'stopRec').name('■ Stop Recording');
  kfFolder.add(kfActions, 'saveKF').name('📷  Save Keyframe  [K]');
  kfFolder.add(kfActions, 'undoKF').name('↩  Undo Last');
  kfFolder.add(kfActions, 'playback').name('▶  Playback');
  kfFolder.add(kfActions, 'downloadVideo').name('↓  Download Video');
  kfFolder.close();

  // Press K to quick-save a keyframe
  document.addEventListener('keydown', (e) => {
    if (e.code === 'KeyK' && !e.ctrlKey && !e.altKey) kfActions.saveKF();
  });


  // ─── Help Modal ──────────────────────────────────────────────────────────
  const showHelp = () => {
    if (document.getElementById('__mujoco_help_modal')) return;
    const overlay = document.createElement('div');
    overlay.id = '__mujoco_help_modal';
    overlay.style.cssText =
      'position:fixed;inset:0;background:rgba(0,0,0,0.72);z-index:2000;' +
      'display:flex;align-items:center;justify-content:center;';

    const modal = document.createElement('div');
    modal.style.cssText =
      'background:#151525;color:#ddd;border-radius:14px;padding:30px 36px;' +
      'max-width:620px;width:92%;max-height:82vh;overflow-y:auto;' +
      'font-family:Arial,sans-serif;box-shadow:0 10px 48px rgba(0,0,0,0.8);border:1px solid #2a2a4a;';

    modal.innerHTML = `
      <h2 style="margin:0 0 16px;font-size:21px;color:#7eb8f7;text-align:center;">
        &#128214; MuJoCo Viewer &#8212; User Guide
      </h2>

      <h3 style="color:#f0a050;margin:14px 0 6px;">&#128432;&#65039; Camera Navigation</h3>
      <table style="width:100%;border-collapse:collapse;font-size:13px;">
        <tr><td style="padding:3px 8px;color:#888;width:52%;">Orbit / Rotate</td><td style="padding:3px 8px;"><b>Left-click</b> + drag</td></tr>
        <tr><td style="padding:3px 8px;color:#888;">Pan</td><td style="padding:3px 8px;"><b>Right-click</b> + drag</td></tr>
        <tr><td style="padding:3px 8px;color:#888;">Zoom</td><td style="padding:3px 8px;"><b>Scroll wheel</b></td></tr>
        <tr><td style="padding:3px 8px;color:#888;">Reset camera</td><td style="padding:3px 8px;"><kbd>Ctrl + A</kbd></td></tr>
      </table>

      <h3 style="color:#f0a050;margin:14px 0 6px;">&#9000;&#65039; Keyboard Shortcuts</h3>
      <table style="width:100%;border-collapse:collapse;font-size:13px;">
        <tr><td style="padding:3px 8px;color:#888;width:52%;">Pause / Resume</td><td style="padding:3px 8px;"><kbd>Space</kbd></td></tr>
        <tr><td style="padding:3px 8px;color:#888;">Reset simulation</td><td style="padding:3px 8px;"><kbd>Backspace</kbd></td></tr>
        <tr><td style="padding:3px 8px;color:#888;">Reload XML</td><td style="padding:3px 8px;"><kbd>Ctrl + L</kbd></td></tr>
        <tr><td style="padding:3px 8px;color:#888;">Save keyframe</td><td style="padding:3px 8px;"><kbd>K</kbd></td></tr>
      </table>

      <h3 style="color:#f0a050;margin:14px 0 6px;">&#129534; Adjusting Joints</h3>
      <p style="font-size:13px;line-height:1.65;margin:0 0 4px;">
        Open <b>&#129534; Joints</b> in the right panel. Joints are grouped by limb:<br>
        <b>Left Leg &nbsp;/&nbsp; Right Leg &nbsp;/&nbsp; Waist &amp; Back &nbsp;/&nbsp; Left Arm &nbsp;/&nbsp; Right Arm</b><br>
        Drag a slider to change a joint angle in real time.
        <span style="color:#f0a050;font-size:12px">&nbsp;&#9432; Pause the simulation first for best control.</span>
      </p>

      <h3 style="color:#f0a050;margin:14px 0 6px;">&#127775; Basic Pose Tips (Unitree G1)</h3>
      <p style="font-size:12px;color:#aaa;margin:0 0 6px;">
        Press <kbd>Space</kbd> to <b>Pause</b>, then open a limb group and adjust sliders. The robot moves instantly.
      </p>

      <details style="margin-bottom:6px;">
        <summary style="font-size:13px;font-weight:bold;color:#c8e6ff;cursor:pointer;padding:4px 0;">
          &#129307; T-Pose (arms horizontal)
        </summary>
        <table style="width:100%;border-collapse:collapse;font-size:12px;margin-top:4px;">
          <tr style="background:rgba(255,255,255,0.05);">
            <th style="padding:3px 8px;text-align:left;color:#888;">Limb group</th>
            <th style="padding:3px 8px;text-align:left;color:#888;">Joint</th>
            <th style="padding:3px 8px;text-align:left;color:#888;">Value</th>
          </tr>
          <tr><td style="padding:2px 8px;color:#aaa;">Left Arm</td><td style="padding:2px 8px;">left shoulder roll</td><td style="padding:2px 8px;color:#6cf;">+1.57</td></tr>
          <tr><td style="padding:2px 8px;color:#aaa;">Right Arm</td><td style="padding:2px 8px;">right shoulder roll</td><td style="padding:2px 8px;color:#6cf;">&#8722;1.57</td></tr>
          <tr><td style="padding:2px 8px;color:#aaa;">Left Arm</td><td style="padding:2px 8px;">left elbow</td><td style="padding:2px 8px;color:#6cf;">0</td></tr>
          <tr><td style="padding:2px 8px;color:#aaa;">Right Arm</td><td style="padding:2px 8px;">right elbow</td><td style="padding:2px 8px;color:#6cf;">0</td></tr>
        </table>
      </details>

      <details style="margin-bottom:6px;">
        <summary style="font-size:13px;font-weight:bold;color:#c8e6ff;cursor:pointer;padding:4px 0;">
          &#128075; Wave (right arm raised)
        </summary>
        <table style="width:100%;border-collapse:collapse;font-size:12px;margin-top:4px;">
          <tr style="background:rgba(255,255,255,0.05);">
            <th style="padding:3px 8px;text-align:left;color:#888;">Limb group</th>
            <th style="padding:3px 8px;text-align:left;color:#888;">Joint</th>
            <th style="padding:3px 8px;text-align:left;color:#888;">Value</th>
          </tr>
          <tr><td style="padding:2px 8px;color:#aaa;">Right Arm</td><td style="padding:2px 8px;">right shoulder pitch</td><td style="padding:2px 8px;color:#6cf;">&#8722;1.2</td></tr>
          <tr><td style="padding:2px 8px;color:#aaa;">Right Arm</td><td style="padding:2px 8px;">right shoulder roll</td><td style="padding:2px 8px;color:#6cf;">&#8722;0.3</td></tr>
          <tr><td style="padding:2px 8px;color:#aaa;">Right Arm</td><td style="padding:2px 8px;">right elbow</td><td style="padding:2px 8px;color:#6cf;">+1.0</td></tr>
        </table>
        <p style="font-size:12px;color:#aaa;margin:4px 8px 0;">
          &#128161; <b>Tip:</b> Save two keyframes &#8212; arm down &amp; arm up &#8212; for a waving animation loop.
        </p>
      </details>

      <details style="margin-bottom:6px;">
        <summary style="font-size:13px;font-weight:bold;color:#c8e6ff;cursor:pointer;padding:4px 0;">
          &#129336; Squat
        </summary>
        <table style="width:100%;border-collapse:collapse;font-size:12px;margin-top:4px;">
          <tr style="background:rgba(255,255,255,0.05);">
            <th style="padding:3px 8px;text-align:left;color:#888;">Limb group</th>
            <th style="padding:3px 8px;text-align:left;color:#888;">Joint</th>
            <th style="padding:3px 8px;text-align:left;color:#888;">Value</th>
          </tr>
          <tr><td style="padding:2px 8px;color:#aaa;">Left Leg</td><td style="padding:2px 8px;">left hip pitch</td><td style="padding:2px 8px;color:#6cf;">&#8722;0.8</td></tr>
          <tr><td style="padding:2px 8px;color:#aaa;">Left Leg</td><td style="padding:2px 8px;">left knee</td><td style="padding:2px 8px;color:#6cf;">+1.4</td></tr>
          <tr><td style="padding:2px 8px;color:#aaa;">Left Leg</td><td style="padding:2px 8px;">left ankle pitch</td><td style="padding:2px 8px;color:#6cf;">&#8722;0.5</td></tr>
          <tr><td style="padding:2px 8px;color:#aaa;">Right Leg</td><td style="padding:2px 8px;">right hip pitch</td><td style="padding:2px 8px;color:#6cf;">&#8722;0.8</td></tr>
          <tr><td style="padding:2px 8px;color:#aaa;">Right Leg</td><td style="padding:2px 8px;">right knee</td><td style="padding:2px 8px;color:#6cf;">+1.4</td></tr>
          <tr><td style="padding:2px 8px;color:#aaa;">Right Leg</td><td style="padding:2px 8px;">right ankle pitch</td><td style="padding:2px 8px;color:#6cf;">&#8722;0.5</td></tr>
          <tr><td style="padding:2px 8px;color:#aaa;">Waist / Back</td><td style="padding:2px 8px;">waist yaw</td><td style="padding:2px 8px;color:#6cf;">0</td></tr>
        </table>
      </details>

      <details style="margin-bottom:6px;">
        <summary style="font-size:13px;font-weight:bold;color:#c8e6ff;cursor:pointer;padding:4px 0;">
          &#128081; Salute
        </summary>
        <table style="width:100%;border-collapse:collapse;font-size:12px;margin-top:4px;">
          <tr style="background:rgba(255,255,255,0.05);">
            <th style="padding:3px 8px;text-align:left;color:#888;">Limb group</th>
            <th style="padding:3px 8px;text-align:left;color:#888;">Joint</th>
            <th style="padding:3px 8px;text-align:left;color:#888;">Value</th>
          </tr>
          <tr><td style="padding:2px 8px;color:#aaa;">Right Arm</td><td style="padding:2px 8px;">right shoulder pitch</td><td style="padding:2px 8px;color:#6cf;">&#8722;0.6</td></tr>
          <tr><td style="padding:2px 8px;color:#aaa;">Right Arm</td><td style="padding:2px 8px;">right shoulder roll</td><td style="padding:2px 8px;color:#6cf;">&#8722;0.2</td></tr>
          <tr><td style="padding:2px 8px;color:#aaa;">Right Arm</td><td style="padding:2px 8px;">right elbow</td><td style="padding:2px 8px;color:#6cf;">+1.5</td></tr>
          <tr><td style="padding:2px 8px;color:#aaa;">Right Arm</td><td style="padding:2px 8px;">right wrist roll</td><td style="padding:2px 8px;color:#6cf;">&#8722;0.3</td></tr>
        </table>
      </details>

      <details style="margin-bottom:10px;">
        <summary style="font-size:13px;font-weight:bold;color:#c8e6ff;cursor:pointer;padding:4px 0;">
          &#128161; General Tips &amp; Tricks
        </summary>
        <ul style="font-size:12px;line-height:1.8;margin:6px 0 0 0;padding-left:18px;color:#ccc;">
          <li>Always <b>Pause</b> the sim before adjusting many sliders — it gives cleaner snapshots.</li>
          <li>Use <b>Backspace</b> to reset to the default standing pose at any time.</li>
          <li>Adjust <b>Transition (s)</b> before recording — shorter = snappier, longer = smoother.</li>
          <li>Save a <b>neutral standing pose</b> as keyframe 1 and a target pose as keyframe 2 for a clean stand&#8209;to&#8209;pose clip.</li>
          <li>Record 3+ keyframes: <em>stand → squat → stand</em> for a loopable animation.</li>
          <li>Use <b>Ctrl + A</b> to reset the camera to the front view before starting Playback for a nice angle.</li>
          <li>The file exports as <code>.mp4</code> (if supported) or <code>.webm</code>.</li>
        </ul>
      </details>

      <h3 style="color:#f0a050;margin:14px 0 6px;">&#127908; Keyframe Recording Workflow</h3>
      <p style="font-size:13px;line-height:1.65;margin:0 0 6px;">
        1. Press <kbd>Space</kbd> to pause. Adjust sliders to your first pose.<br>
        2. Click <b>&#9679; Start Recording</b> &#8212; the red <code>&#9679; REC</code> badge appears.<br>
        3. Click <b>&#128247; Save Keyframe</b> (or <kbd>K</kbd>) to store the current pose.<br>
        4. Adjust sliders to a new pose &#8594; press <kbd>K</kbd> again. Repeat as needed.<br>
        5. Click <b>&#9632; Stop Recording</b> to end the session.<br>
        6a. <b>&#9654; Playback</b> &#8212; preview the animation in the browser.<br>
        6b. <b>&#8595; Download Video</b> &#8212; exports as <code>.mp4</code> or <code>.webm</code>.
      </p>

      <h3 style="color:#f0a050;margin:14px 0 6px;">&#128432;&#65039; Dragging Robot Bodies</h3>
      <p style="font-size:13px;line-height:1.65;margin:0;">
        <b>Left-click and drag</b> any body part to apply a force to it.<br>
        Works best while the simulation is running (not paused).
      </p>

      <button id="__mujoco_help_close" style="
        display:block;margin:22px auto 0;padding:9px 30px;
        background:#7eb8f7;color:#111;border:none;border-radius:8px;
        font-size:14px;font-weight:bold;cursor:pointer;">
        &#10005; Close
      </button>`;

    overlay.appendChild(modal);
    document.body.appendChild(overlay);
    document.getElementById('__mujoco_help_close').onclick = () => overlay.remove();
    overlay.addEventListener('click', (e) => { if (e.target === overlay) overlay.remove(); });
  };

  const helpFolder = parentContext.gui.addFolder('&#10067; Help');
  helpFolder.add({ show: showHelp }, 'show').name('&#128214; View User Guide');

  parentContext.gui.open();
}


/** Loads a scene for MuJoCo
 * @param {mujoco} mujoco This is a reference to the mujoco namespace object
 * @param {string} filename This is the name of the .xml file in the /working/ directory of the MuJoCo/Emscripten Virtual File System
 * @param {MuJoCoDemo} parent The three.js Scene Object to add the MuJoCo model elements to
 */
export async function loadSceneFromURL(mujoco, filename, parent) {
  // Free the old data.
  if (parent.data != null) {
    parent.data.delete();
    parent.model = null;
    parent.data = null;
  }

  // Load in the state from XML.
  parent.model = mujoco.MjModel.loadFromXML("/working/" + filename);
  parent.data = new mujoco.MjData(parent.model);

  let model = parent.model;
  let data = parent.data;

  // Decode the null-terminated string names.
  let textDecoder = new TextDecoder("utf-8");
  let names_array = new Uint8Array(model.names);
  let fullString = textDecoder.decode(model.names);
  let names = fullString.split(textDecoder.decode(new ArrayBuffer(1)));

  // Create the root object.
  let mujocoRoot = new THREE.Group();
  mujocoRoot.name = "MuJoCo Root";
  parent.scene.add(mujocoRoot);

  /** @type {Object.<number, THREE.Group>} */
  let bodies = {};
  /** @type {Object.<number, THREE.BufferGeometry>} */
  let meshes = {};
  /** @type {THREE.Light[]} */
  let lights = [];

  // Default material definition.
  let material = new THREE.MeshPhysicalMaterial();
  material.color = new THREE.Color(1, 1, 1);

  // Loop through the MuJoCo geoms and recreate them in three.js.
  for (let g = 0; g < model.ngeom; g++) {
    // Only visualize geom groups up to 2 (same default behavior as simulate).
    if (!(model.geom_group[g] < 3)) { continue; }

    // Get the body ID and type of the geom.
    let b = model.geom_bodyid[g];
    let type = model.geom_type[g];
    let size = [
      model.geom_size[(g * 3) + 0],
      model.geom_size[(g * 3) + 1],
      model.geom_size[(g * 3) + 2]
    ];

    // Create the body if it doesn't exist.
    if (!(b in bodies)) {
      bodies[b] = new THREE.Group();

      let start_idx = model.name_bodyadr[b];
      let end_idx = start_idx;
      while (end_idx < names_array.length && names_array[end_idx] !== 0) {
        end_idx++;
      }
      let name_buffer = names_array.subarray(start_idx, end_idx);
      bodies[b].name = textDecoder.decode(name_buffer);

      bodies[b].bodyID = b;
      bodies[b].has_custom_mesh = false;
    }

    // Set the default geometry. In MuJoCo, this is a sphere.
    let geometry = new THREE.SphereGeometry(size[0] * 0.5);
    if (type == mujoco.mjtGeom.mjGEOM_PLANE.value) {
      // Special handling for plane later.
    } else if (type == mujoco.mjtGeom.mjGEOM_HFIELD.value) {
      // TODO: Implement this.
    } else if (type == mujoco.mjtGeom.mjGEOM_SPHERE.value) {
      geometry = new THREE.SphereGeometry(size[0]);
    } else if (type == mujoco.mjtGeom.mjGEOM_CAPSULE.value) {
      geometry = new THREE.CapsuleGeometry(size[0], size[1] * 2.0, 20, 20);
    } else if (type == mujoco.mjtGeom.mjGEOM_ELLIPSOID.value) {
      geometry = new THREE.SphereGeometry(1); // Stretch this below
    } else if (type == mujoco.mjtGeom.mjGEOM_CYLINDER.value) {
      geometry = new THREE.CylinderGeometry(size[0], size[0], size[1] * 2.0);
    } else if (type == mujoco.mjtGeom.mjGEOM_BOX.value) {
      geometry = new THREE.BoxGeometry(size[0] * 2.0, size[2] * 2.0, size[1] * 2.0);
    } else if (type == mujoco.mjtGeom.mjGEOM_MESH.value) {
      let meshID = model.geom_dataid[g];

      if (!(meshID in meshes)) {
        geometry = new THREE.BufferGeometry();

        let vertex_buffer = model.mesh_vert.subarray(
          model.mesh_vertadr[meshID] * 3,
          (model.mesh_vertadr[meshID] + model.mesh_vertnum[meshID]) * 3);
        for (let v = 0; v < vertex_buffer.length; v += 3) {
          //vertex_buffer[v + 0] =  vertex_buffer[v + 0];
          let temp = vertex_buffer[v + 1];
          vertex_buffer[v + 1] = vertex_buffer[v + 2];
          vertex_buffer[v + 2] = -temp;
        }

        let normal_buffer = model.mesh_normal.subarray(
          model.mesh_normaladr[meshID] * 3,
          (model.mesh_normaladr[meshID] + model.mesh_normalnum[meshID]) * 3);
        for (let v = 0; v < normal_buffer.length; v += 3) {
          //normal_buffer[v + 0] =  normal_buffer[v + 0];
          let temp = normal_buffer[v + 1];
          normal_buffer[v + 1] = normal_buffer[v + 2];
          normal_buffer[v + 2] = -temp;
        }

        let uv_buffer = model.mesh_texcoord.subarray(
          model.mesh_texcoordadr[meshID] * 2,
          (model.mesh_texcoordadr[meshID] + model.mesh_texcoordnum[meshID]) * 2);

        let face_to_vertex_buffer = model.mesh_face.subarray(
          model.mesh_faceadr[meshID] * 3,
          (model.mesh_faceadr[meshID] + model.mesh_facenum[meshID]) * 3);
        let face_to_uv_buffer = model.mesh_facetexcoord.subarray(
          model.mesh_faceadr[meshID] * 3,
          (model.mesh_faceadr[meshID] + model.mesh_facenum[meshID]) * 3);
        let face_to_normal_buffer = model.mesh_facenormal.subarray(
          model.mesh_faceadr[meshID] * 3,
          (model.mesh_faceadr[meshID] + model.mesh_facenum[meshID]) * 3);

        // The UV and Normal Buffers are actually indexed by the triangle indices through the face_to_uv_buffer and face_to_normal_buffer.
        // We need to swizzle them into a per-vertex format for three.js
        let swizzled_uv_buffer = new Float32Array((vertex_buffer.length / 3) * 2);
        let swizzled_normal_buffer = new Float32Array(vertex_buffer.length);
        for (let t = 0; t < face_to_vertex_buffer.length / 3; t++) {
          let vi0 = face_to_vertex_buffer[(t * 3) + 0];
          let vi1 = face_to_vertex_buffer[(t * 3) + 1];
          let vi2 = face_to_vertex_buffer[(t * 3) + 2];
          let uvi0 = face_to_uv_buffer[(t * 3) + 0];
          let uvi1 = face_to_uv_buffer[(t * 3) + 1];
          let uvi2 = face_to_uv_buffer[(t * 3) + 2];
          let nvi0 = face_to_normal_buffer[(t * 3) + 0];
          let nvi1 = face_to_normal_buffer[(t * 3) + 1];
          let nvi2 = face_to_normal_buffer[(t * 3) + 2];
          swizzled_uv_buffer[(vi0 * 2) + 0] = uv_buffer[(uvi0 * 2) + 0];
          swizzled_uv_buffer[(vi0 * 2) + 1] = uv_buffer[(uvi0 * 2) + 1];
          swizzled_uv_buffer[(vi1 * 2) + 0] = uv_buffer[(uvi1 * 2) + 0];
          swizzled_uv_buffer[(vi1 * 2) + 1] = uv_buffer[(uvi1 * 2) + 1];
          swizzled_uv_buffer[(vi2 * 2) + 0] = uv_buffer[(uvi2 * 2) + 0];
          swizzled_uv_buffer[(vi2 * 2) + 1] = uv_buffer[(uvi2 * 2) + 1];
          swizzled_normal_buffer[(vi0 * 3) + 0] = normal_buffer[(nvi0 * 3) + 0];
          swizzled_normal_buffer[(vi0 * 3) + 1] = normal_buffer[(nvi0 * 3) + 1];
          swizzled_normal_buffer[(vi0 * 3) + 2] = normal_buffer[(nvi0 * 3) + 2];
          swizzled_normal_buffer[(vi1 * 3) + 0] = normal_buffer[(nvi1 * 3) + 0];
          swizzled_normal_buffer[(vi1 * 3) + 1] = normal_buffer[(nvi1 * 3) + 1];
          swizzled_normal_buffer[(vi1 * 3) + 2] = normal_buffer[(nvi1 * 3) + 2];
          swizzled_normal_buffer[(vi2 * 3) + 0] = normal_buffer[(nvi2 * 3) + 0];
          swizzled_normal_buffer[(vi2 * 3) + 1] = normal_buffer[(nvi2 * 3) + 1];
          swizzled_normal_buffer[(vi2 * 3) + 2] = normal_buffer[(nvi2 * 3) + 2];
        }
        geometry.setAttribute("position", new THREE.BufferAttribute(vertex_buffer, 3));
        geometry.setAttribute("normal", new THREE.BufferAttribute(swizzled_normal_buffer, 3));
        geometry.setAttribute("uv", new THREE.BufferAttribute(swizzled_uv_buffer, 2));
        geometry.setIndex(Array.from(face_to_vertex_buffer));
        geometry.computeVertexNormals(); // MuJoCo Normals acting strangely... just recompute them
        meshes[meshID] = geometry;
      } else {
        geometry = meshes[meshID];
      }

      bodies[b].has_custom_mesh = true;
    }
    // Done with geometry creation.

    // Set the Material Properties of incoming bodies
    let texture = undefined;
    let color = [
      model.geom_rgba[(g * 4) + 0],
      model.geom_rgba[(g * 4) + 1],
      model.geom_rgba[(g * 4) + 2],
      model.geom_rgba[(g * 4) + 3]];
    if (model.geom_matid[g] != -1) {
      let matId = model.geom_matid[g];
      color = [
        model.mat_rgba[(matId * 4) + 0],
        model.mat_rgba[(matId * 4) + 1],
        model.mat_rgba[(matId * 4) + 2],
        model.mat_rgba[(matId * 4) + 3]];

      // Construct Texture from model.tex_data
      texture = undefined;
      // mat_texid is now a matrix (nmat x mjNTEXROLE)
      // We use mjTEXROLE_RGB (value 1) for standard diffuse/color textures
      const mjNTEXROLE = 10; // Total number of texture roles
      const mjTEXROLE_RGB = 1; // RGB texture role
      let texId = model.mat_texid[(matId * mjNTEXROLE) + mjTEXROLE_RGB];

      if (texId != -1) {
        let width = model.tex_width[texId];
        let height = model.tex_height[texId];
        let offset = model.tex_adr[texId];
        let channels = model.tex_nchannel[texId];
        let texData = model.tex_data;
        let rgbaArray = new Uint8Array(width * height * 4);
        for (let p = 0; p < width * height; p++) {
          rgbaArray[(p * 4) + 0] = texData[offset + ((p * channels) + 0)];
          rgbaArray[(p * 4) + 1] = channels > 1 ? texData[offset + ((p * channels) + 1)] : rgbaArray[(p * 4) + 0];
          rgbaArray[(p * 4) + 2] = channels > 2 ? texData[offset + ((p * channels) + 2)] : rgbaArray[(p * 4) + 0];
          rgbaArray[(p * 4) + 3] = channels > 3 ? texData[offset + ((p * channels) + 3)] : 255;
        }
        texture = new THREE.DataTexture(rgbaArray, width, height, THREE.RGBAFormat, THREE.UnsignedByteType);
        if (texId == 2) {
          texture.repeat = new THREE.Vector2(50, 50);
          texture.wrapS = THREE.RepeatWrapping;
          texture.wrapT = THREE.RepeatWrapping;
        } else {
          texture.repeat = new THREE.Vector2(model.mat_texrepeat[(model.geom_matid[g] * 2) + 0],
            model.mat_texrepeat[(model.geom_matid[g] * 2) + 1]);
          texture.wrapS = THREE.RepeatWrapping;
          texture.wrapT = THREE.RepeatWrapping;
        }

        texture.needsUpdate = true;
      }
    }

    // Create a new material for each geom to avoid cross-contamination
    let currentMaterial = new THREE.MeshPhysicalMaterial({
      color: new THREE.Color(color[0], color[1], color[2]),
      transparent: color[3] < 1.0,
      opacity: color[3] / 255.,
      specularIntensity: model.geom_matid[g] != -1 ? model.mat_specular[model.geom_matid[g]] : undefined,
      reflectivity: model.geom_matid[g] != -1 ? model.mat_reflectance[model.geom_matid[g]] : undefined,
      roughness: model.geom_matid[g] != -1 ? 1.0 - model.mat_shininess[model.geom_matid[g]] : undefined,
      metalness: model.geom_matid[g] != -1 ? 0.1 : undefined, //model.mat_metallic   [model.geom_matid[g]]
      map: texture
    });

    let mesh;// = new THREE.Mesh();
    if (type == 0) {
      mesh = new Reflector(new THREE.PlaneGeometry(100, 100), { clipBias: 0.003, texture: texture });
      mesh.rotateX(- Math.PI / 2);
    } else {
      mesh = new THREE.Mesh(geometry, currentMaterial);
    }

    mesh.castShadow = g == 0 ? false : true;
    mesh.receiveShadow = type != 7;
    mesh.bodyID = b;
    bodies[b].add(mesh);
    getPosition(model.geom_pos, g, mesh.position);
    if (type != 0) { getQuaternion(model.geom_quat, g, mesh.quaternion); }
    if (type == 4) { mesh.scale.set(size[0], size[2], size[1]); } // Stretch the Ellipsoid
  }

  // Parse tendons.
  let tendonMat = new THREE.MeshPhongMaterial();
  tendonMat.color = new THREE.Color(0.8, 0.3, 0.3);
  mujocoRoot.cylinders = new THREE.InstancedMesh(
    new THREE.CylinderGeometry(1, 1, 1),
    tendonMat, 1023);
  mujocoRoot.cylinders.receiveShadow = true;
  mujocoRoot.cylinders.castShadow = true;
  mujocoRoot.add(mujocoRoot.cylinders);
  mujocoRoot.spheres = new THREE.InstancedMesh(
    new THREE.SphereGeometry(1, 10, 10),
    tendonMat, 1023);
  mujocoRoot.spheres.receiveShadow = true;
  mujocoRoot.spheres.castShadow = true;
  mujocoRoot.add(mujocoRoot.spheres);

  // Parse lights.
  for (let l = 0; l < model.nlight; l++) {
    let light = new THREE.DirectionalLight();
    if (model.light_type[l] == 0) {
      light = new THREE.SpotLight();
      light.angle = 1.51;//model.light_cutoffangle[l];
    } else if (model.light_type[l] == 1) {
      light = new THREE.DirectionalLight();
    } else if (model.light_type[l] == 2) {
      light = new THREE.PointLight();
    } else if (model.light_type[l] == 3) {
      light = new THREE.HemisphereLight();
    }

    light.angle = 1.11;

    light.decay = model.light_attenuation[l] * 100;
    light.penumbra = 0.5;
    light.castShadow = true; // default false
    light.intensity = light.intensity * 3.14 * 1.0;

    light.shadow.mapSize.width = 1024; // default
    light.shadow.mapSize.height = 1024; // default
    light.shadow.camera.near = 0.1; // default
    light.shadow.camera.far = 10; // default
    //bodies[model.light_bodyid()].add(light);
    if (bodies[0]) {
      bodies[0].add(light);
    } else {
      mujocoRoot.add(light);
    }
    lights.push(light);
  }
  if (model.nlight == 0) {
    let light = new THREE.DirectionalLight();
    mujocoRoot.add(light);
  }

  for (let b = 0; b < model.nbody; b++) {
    //let parent_body = model.body_parentid()[b];
    if (b == 0 || !bodies[0]) {
      mujocoRoot.add(bodies[b]);
    } else if (bodies[b]) {
      bodies[0].add(bodies[b]);
    } else {
      console.log("Body without Geometry detected; adding to bodies", b, bodies[b]);
      bodies[b] = new THREE.Group(); bodies[b].name = names[b + 1]; bodies[b].bodyID = b; bodies[b].has_custom_mesh = false;
      bodies[0].add(bodies[b]);
    }
  }

  parent.mujocoRoot = mujocoRoot;

  return [model, data, bodies, lights];
}

export function drawTendonsAndFlex(mujocoRoot, model, data) {
  // Update tendon transforms.
  let identityQuat = new THREE.Quaternion();
  let numWraps = 0;
  if (mujocoRoot && mujocoRoot.cylinders) {
    let mat = new THREE.Matrix4();
    for (let t = 0; t < model.ntendon; t++) {
      let startW = data.ten_wrapadr[t];
      let r = model.tendon_width[t];
      for (let w = startW; w < startW + data.ten_wrapnum[t] - 1; w++) {
        let tendonStart = getPosition(data.wrap_xpos, w, new THREE.Vector3());
        let tendonEnd = getPosition(data.wrap_xpos, w + 1, new THREE.Vector3());
        let tendonAvg = new THREE.Vector3().addVectors(tendonStart, tendonEnd).multiplyScalar(0.5);

        let validStart = tendonStart.length() > 0.01;
        let validEnd = tendonEnd.length() > 0.01;

        if (validStart) { mujocoRoot.spheres.setMatrixAt(numWraps, mat.compose(tendonStart, identityQuat, new THREE.Vector3(r, r, r))); }
        if (validEnd) { mujocoRoot.spheres.setMatrixAt(numWraps + 1, mat.compose(tendonEnd, identityQuat, new THREE.Vector3(r, r, r))); }
        if (validStart && validEnd) {
          mat.compose(tendonAvg, identityQuat.setFromUnitVectors(
            new THREE.Vector3(0, 1, 0), tendonEnd.clone().sub(tendonStart).normalize()),
            new THREE.Vector3(r, tendonStart.distanceTo(tendonEnd), r));
          mujocoRoot.cylinders.setMatrixAt(numWraps, mat);
          numWraps++;
        }
      }
    }

    let curFlexSphereInd = numWraps;
    let tempvertPos = new THREE.Vector3();
    let tempvertRad = new THREE.Vector3();
    for (let i = 0; i < model.nflex; i++) {
      for (let j = 0; j < model.flex_vertnum[i]; j++) {
        let vertIndex = model.flex_vertadr[i] + j;
        getPosition(data.flexvert_xpos, vertIndex, tempvertPos);
        let r = 0.01;
        mat.compose(tempvertPos, identityQuat, tempvertRad.set(r, r, r));

        mujocoRoot.spheres.setMatrixAt(curFlexSphereInd, mat);
        curFlexSphereInd++;
      }
    }
    mujocoRoot.cylinders.count = numWraps;
    mujocoRoot.spheres.count = curFlexSphereInd;
    mujocoRoot.cylinders.instanceMatrix.needsUpdate = true;
    mujocoRoot.spheres.instanceMatrix.needsUpdate = true;
  }
}

/** Downloads the scenes/assets folder to MuJoCo's virtual filesystem
 * @param {mujoco} mujoco */
/** Downloads the scenes/assets folder to MuJoCo's virtual filesystem
 * @param {mujoco} mujoco */
export async function downloadExampleScenesFolder(mujoco) {
  let allFiles = await (await fetch("./assets/scenes/index.json", { cache: "no-store" })).json();

  // Download in batches to avoid overwhelming the browser
  const BATCH_SIZE = 10;
  for (let i = 0; i < allFiles.length; i += BATCH_SIZE) {
    const batch = allFiles.slice(i, i + BATCH_SIZE);
    // Add a delay between batches to let the network stack breathe
    if (i > 0) await new Promise(r => setTimeout(r, 500));
    console.log(`Starting batch ${Math.floor(i / BATCH_SIZE) + 1}...`);
    const requests = batch.map((url) => fetch("./assets/scenes/" + url, { cache: "no-store" }));
    const responses = await Promise.all(requests);

    for (let j = 0; j < responses.length; j++) {
      let fileIndex = i + j;
      let filename = allFiles[fileIndex];
      let split = filename.split("/");
      let working = '/working/';
      for (let f = 0; f < split.length - 1; f++) {
        working += split[f];
        if (!mujoco.FS.analyzePath(working).exists) { mujoco.FS.mkdir(working); }
        working += "/";
      }

      if (filename.toLowerCase().endsWith(".png") || filename.toLowerCase().endsWith(".stl") || filename.toLowerCase().endsWith(".skn")) {
        mujoco.FS.writeFile("/working/" + filename, new Uint8Array(await responses[j].arrayBuffer()));
      } else {
        mujoco.FS.writeFile("/working/" + filename, await responses[j].text());
      }
      console.log(`Loaded ${fileIndex + 1}/${allFiles.length}: ${filename}`);
    }
  }
  console.log("All assets downloaded and written to VFS.");
}

/** Access the vector at index, swizzle for three.js, and apply to the target THREE.Vector3
 * @param {Float32Array|Float64Array} buffer
 * @param {number} index
 * @param {THREE.Vector3} target */
export function getPosition(buffer, index, target, swizzle = true) {
  if (swizzle) {
    return target.set(
      buffer[(index * 3) + 0],
      buffer[(index * 3) + 2],
      -buffer[(index * 3) + 1]);
  } else {
    return target.set(
      buffer[(index * 3) + 0],
      buffer[(index * 3) + 1],
      buffer[(index * 3) + 2]);
  }
}

/** Access the quaternion at index, swizzle for three.js, and apply to the target THREE.Quaternion
 * @param {Float32Array|Float64Array} buffer
 * @param {number} index
 * @param {THREE.Quaternion} target */
export function getQuaternion(buffer, index, target, swizzle = true) {
  if (swizzle) {
    return target.set(
      -buffer[(index * 4) + 1],
      -buffer[(index * 4) + 3],
      buffer[(index * 4) + 2],
      -buffer[(index * 4) + 0]);
  } else {
    return target.set(
      buffer[(index * 4) + 0],
      buffer[(index * 4) + 1],
      buffer[(index * 4) + 2],
      buffer[(index * 4) + 3]);
  }
}

/** Converts this Vector3's Handedness to MuJoCo's Coordinate Handedness
 * @param {THREE.Vector3} target */
export function toMujocoPos(target) { return target.set(target.x, -target.z, target.y); }

/** Standard normal random number generator using Box-Muller transform */
export function standardNormal() {
  return Math.sqrt(-2.0 * Math.log(Math.random())) *
    Math.cos(2.0 * Math.PI * Math.random());
}
