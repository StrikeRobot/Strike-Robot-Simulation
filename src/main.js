
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

