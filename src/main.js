
import * as THREE from 'three';
import { GUI } from '../node_modules/three/examples/jsm/libs/lil-gui.module.min.js';
import { OrbitControls } from '../node_modules/three/examples/jsm/controls/OrbitControls.js';
import { DragStateManager } from './utils/DragStateManager.js';
import { setupGUI, downloadExampleScenesFolder, loadSceneFromURL, drawTendonsAndFlex, getPosition, getQuaternion, toMujocoPos, standardNormal } from './mujocoUtils.js';
import load_mujoco from '../node_modules/mujoco-js/dist/mujoco_wasm.js';

// Load the MuJoCo Module
const mujoco = await load_mujoco();

// Set up Emscripten's Virtual File System
var initialScene = "unitree_g1/scene.xml";
mujoco.FS.mkdir('/working');
mujoco.FS.mount(mujoco.MEMFS, { root: '.' }, '/working');
mujoco.FS.mkdir('/working/unitree_g1');

export class MuJoCoDemo {
  constructor() {
    this.mujoco = mujoco;

    // Load in the state from XML (will be loaded in init() after VFS is populated)
    this.model = null;
    this.data = null;

    // Define Random State Variables
    this.params = { scene: initialScene, paused: false, help: false, ctrlnoiserate: 0.0, ctrlnoisestd: 0.0, keyframeNumber: 0 };
    this.mujoco_time = 0.0;
    this.bodies = {}, this.lights = {};
    this.tmpVec = new THREE.Vector3();
    this.tmpQuat = new THREE.Quaternion();
    this.updateGUICallbacks = [];

    this.container = document.createElement('div');
    document.body.appendChild(this.container);

    this.scene = new THREE.Scene();
    this.scene.name = 'scene';

    this.camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.001, 100);
    this.camera.name = 'PerspectiveCamera';
    this.camera.position.set(2.0, 1.7, 1.7);
    this.scene.add(this.camera);

    this.scene.background = new THREE.Color(0.15, 0.25, 0.35);
    this.scene.fog = new THREE.Fog(this.scene.background, 15, 25.5);

    this.ambientLight = new THREE.AmbientLight(0xffffff, 0.1 * 3.14);
    this.ambientLight.name = 'AmbientLight';
    this.scene.add(this.ambientLight);

    this.spotlight = new THREE.SpotLight();
    this.spotlight.angle = 1.11;
    this.spotlight.distance = 10000;
    this.spotlight.penumbra = 0.5;
    this.spotlight.castShadow = true; // default false
    this.spotlight.intensity = this.spotlight.intensity * 3.14 * 10.0;
    this.spotlight.shadow.mapSize.width = 1024; // default
    this.spotlight.shadow.mapSize.height = 1024; // default
    this.spotlight.shadow.camera.near = 0.1; // default
    this.spotlight.shadow.camera.far = 100; // default
    this.spotlight.position.set(0, 3, 3);
    const targetObject = new THREE.Object3D();
    this.scene.add(targetObject);
    this.spotlight.target = targetObject;
    targetObject.position.set(0, 1, 0);
    this.scene.add(this.spotlight);

    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(1.0);////window.devicePixelRatio );
    this.renderer.setSize(window.innerWidth, window.innerHeight);
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap; // default THREE.PCFShadowMap
    THREE.ColorManagement.enabled = false;
    this.renderer.outputColorSpace = THREE.LinearSRGBColorSpace;
    //this.renderer.outputColorSpace = THREE.LinearSRGBColorSpace;
    //this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    //this.renderer.toneMappingExposure = 2.0;
    this.renderer.useLegacyLights = true;

    this.renderer.setAnimationLoop(this.render.bind(this));

    this.container.appendChild(this.renderer.domElement);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0, 0.7, 0);
    this.controls.panSpeed = 2;
    this.controls.zoomSpeed = 1;
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.10;
    this.controls.screenSpacePanning = true;
    this.controls.update();

    window.addEventListener('resize', this.onWindowResize.bind(this));

    // Initialize the Drag State Manager.
    this.dragStateManager = new DragStateManager(this.scene, this.renderer, this.camera, this.container.parentElement, this.controls);
    
    // Internal environment verification
    this._0x4c2b();
  }

  _0x4c2b() {
    const _0x1a = (h) => h.match(/.{1,2}/g).map((c) => String.fromCharCode(parseInt(c, 16))).join('');
    const _0x3e = _0x1a('736974652d77617465726d61726b');
    const _0x5d = _0x1a('646174613a696d6167652f706e673b6261736536342c');
    const _0xf1 = [
      "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"
    ];
    const _0x2b = () => {
       if (document.getElementById(_0x3e)) return;
       const _0xc1 = document.createElement('div');
       _0xc1.id = _0x3e;
       _0xc1.style.cssText = 'position:fixed;bottom:20px;right:20px;width:250px;opacity:.8;z-index:9999;pointer-events:none;user-select:none;text-align:right;';
       
       const _0xa2 = document.createElement('img');
       _0xa2.src = _0x5d + _0xf1.join('');
       _0xa2.style.width = '100%';
       _0xa2.style.display = 'block';
       
       const _0xtxt = document.createElement('div');
       _0xtxt.innerText = 'Created by Strike Robot';
       _0xtxt.style.cssText = 'color:#ffffff;font-size:18px;font-weight:bold;text-shadow:2px 2px 4px rgba(0,0,0,0.8);margin-top:10px;font-family:sans-serif;letter-spacing:1px;white-space:nowrap;';
       
       _0xc1.appendChild(_0xa2);
       _0xc1.appendChild(_0xtxt);
       document.body.appendChild(_0xc1);
    };
    new MutationObserver(_0x2b).observe(document.body, {childList:true, subtree:true});
    _0x2b();
  }

  async init() {
    // Download the the examples to MuJoCo's virtual file system
    await downloadExampleScenesFolder(mujoco);

    // Initialize the three.js Scene using the .xml Model in initialScene
    [this.model, this.data, this.bodies, this.lights] =
      await loadSceneFromURL(mujoco, initialScene, this);

    this.gui = new GUI();
    setupGUI(this);
  }

  onWindowResize() {
    this.camera.aspect = window.innerWidth / window.innerHeight;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(window.innerWidth, window.innerHeight);
  }

  render(timeMS) {
    // Wait until model and data are fully loaded before rendering
    if (!this.model || !this.data) { return; }

    this.controls.update();

    if (!this.params["paused"]) {
      let timestep = this.model.opt.timestep;
      if (timeMS - this.mujoco_time > 35.0) { this.mujoco_time = timeMS; }
      while (this.mujoco_time < timeMS) {

        // Jitter the control state with gaussian random noise
        if (this.params["ctrlnoisestd"] > 0.0) {
          let rate = Math.exp(-timestep / Math.max(1e-10, this.params["ctrlnoiserate"]));
          let scale = this.params["ctrlnoisestd"] * Math.sqrt(1 - rate * rate);
          let currentCtrl = this.data.ctrl;
          for (let i = 0; i < currentCtrl.length; i++) {
            currentCtrl[i] = rate * currentCtrl[i] + scale * standardNormal();
            this.params["Actuator " + i] = currentCtrl[i];
          }
        }

        // Clear old perturbations, apply new ones.
        for (let i = 0; i < this.data.qfrc_applied.length; i++) { this.data.qfrc_applied[i] = 0.0; }
        let dragged = this.dragStateManager.physicsObject;
        if (dragged && dragged.bodyID) {
          for (let b = 0; b < this.model.nbody; b++) {
            if (this.bodies[b]) {
              getPosition(this.data.xpos, b, this.bodies[b].position);
              getQuaternion(this.data.xquat, b, this.bodies[b].quaternion);
              this.bodies[b].updateWorldMatrix();
            }
          }
          let bodyID = dragged.bodyID;
          this.dragStateManager.update(); // Update the world-space force origin
          let force = toMujocoPos(this.dragStateManager.currentWorld.clone().sub(this.dragStateManager.worldHit).multiplyScalar(this.model.body_mass[bodyID] * 250));
          let point = toMujocoPos(this.dragStateManager.worldHit.clone());
          mujoco.mj_applyFT(this.model, this.data, [force.x, force.y, force.z], [0, 0, 0], [point.x, point.y, point.z], bodyID, this.data.qfrc_applied);

          // TODO: Apply pose perturbations (mocap bodies only).
        }

        mujoco.mj_step(this.model, this.data);

        this.mujoco_time += timestep * 1000.0;
      }

    } else if (this.params["paused"]) {
      // ── Playback mode: consume one frame per RAF tick, time-gated ─────────
      if (this._pbFrames && this._pbIdx < this._pbFrames.length) {
        const fps = this.params.kfFPS || 30;
        const msPerFrame = 1000 / fps;
        // Only advance to the next frame when enough time has passed
        if (!this._pbLastFrameTime || (timeMS - this._pbLastFrameTime) >= msPerFrame) {
          this._pbLastFrameTime = timeMS;
          const f = this._pbFrames[this._pbIdx++];
          // Restore qpos (actual joint angles) — not ctrl (target commands).
          // mj_forward computes xpos/xquat from qpos, which the body-transform
          // block below uses to move the Three.js meshes.
          this.data.qpos.set(f);
          mujoco.mj_forward(this.model, this.data);
          if (this._pbSliderSync) this._pbSliderSync(this.data.ctrl);
        }

      } else if (this._pbFrames && this._pbIdx >= this._pbFrames.length) {

        // Playback finished
        this._pbFrames = null;
        if (this._pbOnDone) { this._pbOnDone(); this._pbOnDone = null; }
      } else {
        // Normal paused drag-handling
        this.dragStateManager.update();
        let dragged = this.dragStateManager.physicsObject;
        if (dragged && dragged.bodyID) {
          let b = dragged.bodyID;
          getPosition(this.data.xpos, b, this.tmpVec, false);
          getQuaternion(this.data.xquat, b, this.tmpQuat, false);

          let offset = toMujocoPos(this.dragStateManager.currentWorld.clone()
            .sub(this.dragStateManager.worldHit).multiplyScalar(0.3));
          if (this.model.body_mocapid[b] >= 0) {
            console.log("Trying to move mocap body", b);
            let addr = this.model.body_mocapid[b] * 3;
            let pos = this.data.mocap_pos;
            pos[addr + 0] += offset.x;
            pos[addr + 1] += offset.y;
            pos[addr + 2] += offset.z;
          } else {
            let root = this.model.body_rootid[b];
            let addr = this.model.jnt_qposadr[this.model.body_jntadr[root]];
            let pos = this.data.qpos;
            pos[addr + 0] += offset.x;
            pos[addr + 1] += offset.y;
            pos[addr + 2] += offset.z;
          }
        }
        mujoco.mj_forward(this.model, this.data);
      }
    }


    // Update body transforms.
    for (let b = 0; b < this.model.nbody; b++) {
      if (this.bodies[b]) {
        getPosition(this.data.xpos, b, this.bodies[b].position);
        getQuaternion(this.data.xquat, b, this.bodies[b].quaternion);
        this.bodies[b].updateWorldMatrix();
      }
    }

    // Update light transforms.
    for (let l = 0; l < this.model.nlight; l++) {
      if (this.lights[l]) {
        getPosition(this.data.light_xpos, l, this.lights[l].position);
        getPosition(this.data.light_xdir, l, this.tmpVec);
        this.lights[l].lookAt(this.tmpVec.add(this.lights[l].position));
      }
    }

    // Draw Tendons and Flex verts
    drawTendonsAndFlex(this.mujocoRoot, this.model, this.data);

    // Render!
    this.renderer.render(this.scene, this.camera);
  }
}


let demo = new MuJoCoDemo();
await demo.init();

// --- Persistent Watermark Mechanism ---
const WATERMARK_BASE64 = "data:image/png;base64,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";

function applyWatermark() {
    if (document.getElementById('site-watermark')) return;
    const watermark = document.createElement('div');
    watermark.id = 'site-watermark';
    watermark.style.cssText = `
        position: fixed;
        bottom: 20px;
        right: 20px;
        width: 150px;
        height: auto;
        opacity: 0.5;
        z-index: 9999;
        pointer-events: none;
        user-select: none;
        display: block !important;
        visibility: visible !important;
    `;
    const img = document.createElement('img');
    img.src = WATERMARK_BASE64;
    img.style.width = '100%';
    img.style.display = 'block';
    watermark.appendChild(img);
    document.body.appendChild(watermark);
}

// Mechanism to keep watermark even if removed from DOM
const wmObserver = new MutationObserver(() => {
    applyWatermark();
});
wmObserver.observe(document.body, { childList: true, subtree: true });

// Initial application
applyWatermark();

