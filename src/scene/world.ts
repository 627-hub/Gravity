import {
  Scene, PerspectiveCamera, WebGLRenderer, Vector3, Color,
  Mesh, SphereGeometry, MeshStandardMaterial, MeshBasicMaterial,
  Group, ConeGeometry, CylinderGeometry, BoxGeometry,
  PointLight, AmbientLight, BufferGeometry, LineBasicMaterial, Line,
  Float32BufferAttribute, AdditiveBlending, BackSide, Points,
  PointsMaterial, RingGeometry, DoubleSide, MathUtils, ArrowHelper,
  LineDashedMaterial, EdgesGeometry, LineSegments, Raycaster, Vector2,
  CatmullRomCurve3, WireframeGeometry,
  type ColorRepresentation,
} from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { CSS2DRenderer, CSS2DObject } from 'three/addons/renderers/CSS2DRenderer.js';

import { ALL_BODIES, PLANETS, SUN, type Body, type Moon } from '../data/bodies';
import { DAY, AU_KM, AU, G, M_SUN } from '../data/constants';
import { keplerState } from '../physics/state';
import { keplerPosition, sampleOrbit, orbitalPeriodDays } from '../physics/kepler';
import { NBody } from '../physics/nbody';
import { NavViz, CRAFT_ID } from './nav-viz';
import { Mission as FlightMission } from '../nav/mission';
import type { TrackingTier } from '../nav/navigator';
import { P_SRP_1AU } from '../nav/perturbations';
import { DEFAULT_SRP } from '../nav/truth';
import type { ThrustDir } from '../nav/propulsion';
import { AUDAY_TO_KMS, fromKms, MU_SUN, toKms } from '../nav/units';
import { elementsFromState, type ClassicalElements } from '../nav/elements';
import { makeForceModel, type AccelFn, type GravitySource } from '../nav/perturbations';
import { Adcs, DEFAULT_ADCS } from '../nav/attitude';

/** 手动点火方向（= 推力方向）。 */
export type ManualBurnDir = ThrustDir;
import type { Spaceport } from '../nav/spaceport';
import { solarSystemSources } from '../nav/sources';
import type { TransferPlan } from '../nav/plan';
import {
  buildSimBodies, descriptorState, moonElements, moonRelativePosition,
  shortestMoonPeriod, pairMu, type SimDescriptor,
} from '../data/system';
import { getScale, TRUE_UNITS_PER_AU, type ScaleMode, type ScaleModel } from './scale';
import { surfaceTexture, ringTexture } from './textures';

// Ecliptic frame (x toward equinox, z north) -> Three.js Y-up scene frame.
function eclToScene(v: Vector3, out: Vector3): Vector3 {
  return out.set(v.x, v.z, -v.y);
}

export type PhysicsMode = 'kepler' | 'nbody';
interface BodyView {
  body: Body;
  mesh: Mesh;
  orbitAU: Vector3[];
  orbitLine: Line | null;
  label: CSS2DObject;
  spin: number;
  opacity: number; // eased 0..1
  // transient per-frame state shared with moon rendering
  curAU: Vector3;
  curScene: Vector3;
}

interface MoonView {
  moon: Moon;
  parent: Body;
  mesh: Mesh;
  orbitRelAU: Vector3[];
  orbitLine: Line;
  label: CSS2DObject;
  spin: number;
  opacity: number;
}

export interface WorldState {
  scaleMode: ScaleMode;
  physics: PhysicsMode;
  twoD: number;
  showOrbits: boolean;
  showLabels: boolean;
  showMoonLabels: boolean; // moon name labels (separate from planet labels)
  showMoons: boolean;
  showSpin: boolean;   // axial self-rotation of bodies
  paused: boolean;
  daysPerSecond: number;
}

/** Everything the navigation console needs, in physics units (AU, AU/day). */
export interface NavSnapshot {
  phase: 'docked' | 'cruise' | 'arrived';
  departureId: string;
  targetId: string;
  simDays: number;
  departureDay: number;
  arrivalDay: number;
  tof: number;
  /** 0..1 along the flight. */
  progress: number;
  /** Truth state of the craft (heliocentric ecliptic). */
  shipAU: Vector3;
  shipSpeedKms: number;
  /** 当前日心距离处的太阳逃逸速度（km/s）与飞船速度比（>1 = 已超太阳逃逸）。 */
  escapeSpeedKms: number;
  escapeFraction: number;
  targetAU: Vector3;
  targetDistKm: number;
  /** Rate of closure with the target, km/s (positive = approaching). */
  closingKms: number;
  /** Trajectories for the mini-map: plan / flown / onboard prediction. */
  planArc: Vector3[];
  flown: Vector3[];
  predicted: Vector3[];
  status: MissionStatus;
  /** ADCS readouts (null before the cruise starts). */
  adcs: {
    pointingArcsec: number;
    estimateArcsec: number;
    rateDegS: number;
    starUpdates: number;
    starSigmaArcsec: number;
    gyroArwDegSqrtH: number;
  } | null;
  /** Attitude: nose held prograde, up = ecliptic north; angles in craft frame. */
  attitude: {
    /** Heading (ecliptic longitude of the nose) / elevation, degrees. */
    yawDeg: number;
    pitchDeg: number;
    /** Angular rate the attitude control must track, deg/day. */
    gyroDegPerDay: number;
    /** Directions of Sun / target / Earth in the craft frame (polar az/el). */
    sun: PointingReading;
    target: PointingReading;
    earth: PointingReading;
  };
  accel: {
    /**
     * Modelled gravitational acceleration at the craft, mm/s^2. Null while
     * parked on a body (or essentially on top of one), where the point-mass
     * readout diverges and is not meaningful anyway.
     */
    gravityMms2: number | null;
    /** Its direction in the craft frame (null alongside gravityMms2). */
    gravity: PointingReading | null;
    /** Onboard accelerometer (non-gravitational), micro-g. */
    nonGravUg: number;
  };
  orbit: ClassicalElements;
  /** Raw instrument readings from the most recent tracking pass. */
  measurement: {
    t: number;
    rangeKm: number | null;
    rangeRateKms: number | null;
    /** Optical bearing to the target, degrees off the nose. */
    opticalOffDeg: number | null;
    rangeSigmaKm: number | null;
    rateSigmaKms: number | null;
    opticalSigmaArcsec: number | null;
  } | null;
}

/** A direction expressed in the craft frame (polar: azimuth off the nose). */
export interface PointingReading {
  /** Azimuth: 0 = straight ahead, ±180 = behind. */
  azDeg: number;
  /** Elevation above the craft's up axis. */
  elDeg: number;
  /** Off-nose angle: 0 = dead ahead. */
  offDeg: number;
  /** Position angle in the transverse plane: 0 = craft right, 90 = up. */
  posDeg: number;
}

/** Mission snapshot handed to the navigation UI each frame. */
export interface MissionStatus {
  departureId: string;
  targetId: string;
  phase: 'docked' | 'cruise' | 'arrived';
  departureDay: number;
  arrivalDay: number;
  tof: number;
  dvDepart: number;
  dvArrive: number;
  dvTotal: number;
  /** 双曲剩余速度 v∞（km/s）——渐近速度，不是点火量。 */
  vinfDepartKms: number;
  vinfArriveKms: number;
  /** 出发/抵达太空港。 */
  departPort: Spaceport | null;
  arrivePort: Spaceport | null;
  daysToDeparture: number;
  daysToArrival: number;
  /** True separation from the rendezvous point at arrival, km. */
  missKm: number;
  /** Separation the onboard navigation expects at arrival, km. */
  estMissKm: number;
  /** Delta-v needed by a correction burn right now, km/s (cruise only). */
  tcmDvKms: number | null;
  tcmCount: number;
  tcmUsedKms: number;
  manualCount: number;
  manualUsedKms: number;
  /** 推进系统与油门读数。 */
  driveLabel: string;
  massKg: number;
  propellantKg: number;
  thrustMms2: number;
  thrustActive: boolean;
  throttleLevel: number;
  /** L1 readout: null when flying without tracking (truth navigation). */
  trackingLabel: string | null;
  trackCount: number;
  /** Onboard estimate error vs truth (god's-eye comparison), km. */
  estErrorKm: number;
  /** Onboard position uncertainty (1-sigma), km. */
  posSigmaKm: number;
}

export class World {
  readonly scene = new Scene();
  readonly camera: PerspectiveCamera;
  readonly renderer: WebGLRenderer;
  private labelRenderer: CSS2DRenderer;
  private controls: OrbitControls;

  private views: BodyView[] = [];
  private moonViews: MoonView[] = [];
  private scale: ScaleModel;

  private nbody!: NBody;
  private simBodies: SimDescriptor[] = [];
  private simIndexByPlanet = new Map<string, number>();
  /** Smallest moon semi-major axis (AU) per planet, for visual exaggeration. */
  private minMoonA = new Map<string, number>();

  private flatten = 0;
  private polarLimit = Math.PI;

  // Smooth camera fly-to between steps. When camPosGoal is set the camera and
  // its target ease toward the goal each frame; user interaction cancels it.
  private camPosGoal: Vector3 | null = null;
  private camTargetGoal = new Vector3();
  // Drag-to-rotate: while dragging, auto-framing is suspended
  // so the user can orbit freely; on release the camera eases back to the
  // home framing (only when returnOnRelease is on).
  private userDragging = false;
  private returnOnRelease = false;
  private wantAutoRotate = false;

  // Free-explore hover: when on, hovering a body reveals its label + orbit.
  private hoverEnabled = true;
  private hoveredId: string | null = null;
  private raycaster = new Raycaster();
  private pointerNDC = new Vector2(2, 2); // off-screen until the mouse moves
  private homeCamPos: Vector3 | null = null;
  private homeCamTarget = new Vector3();

  // Camera follow: keeps a moving body framed (e.g. the Earth–Moon system,
  // since Earth itself orbits the Sun). Offsets are relative to the body.
  private followId: string | null = null;
  private followCamOffset = new Vector3();
  private followTgtOffset = new Vector3();
  private followCamPos = new Vector3();
  private followLast = new Vector3();   // followed body's previous scene pos
  private followHasLast = false;
  /** User has zoomed/orbited while following: keep their pose, translate only. */
  private followUserAdjusted = false;
  private followDelta = new Vector3();

  /** When non-null, only bodies whose id is present are shown. */
  visible: Set<string> | null = null;

  // Navigation mission: a planned transfer flown by the spacecraft.
  private navViz!: NavViz;
  private mission: FlightMission | null = null;
  private missionPhase: 'docked' | 'cruise' | 'arrived' = 'docked';
  private missionTrackingLabel: string | null = null;
  /** Cached force model + sources for the console's gravity readout. */
  private navForceModel: AccelFn | null = null;
  private navSources: GravitySource[] | null = null;
  /** ADCS: attitude dynamics + gyro/star-tracker navigation. */
  private adcs: Adcs | null = null;
  private adcsDay = 0;
  private adcsPointingArcsec = 0;

  simDays = 0;
  energy0 = 0;

  state: WorldState = {
    scaleMode: 'visual',
    physics: 'kepler',
    twoD: 0,
    showOrbits: true,
    showLabels: true,
    showMoonLabels: true,
    showMoons: false,
    showSpin: true,
    paused: false,
    daysPerSecond: 20,
  };

  // Astrodynamics overlays (spheres of influence, gravity-assist trajectories).
  // SOI: nested spheres — the Sun's, Earth's (on its orbit), and the Moon's.
  /** 太空港示意图（不是真实尺度）：同步/停泊轨道环 + 标签。 */
  private portRings: { ring: Line; label: CSS2DObject }[] = [];
  private readonly portTmp = new Vector3();
  // Gravity assist (Voyager 1 & 2): paths rebuilt in scene units at slide start.
  // Spacetime-curvature slide: a warped grid (the "fabric"), a central mass that
  // dents it, and a body rolling around the well.
  private sunTime = { value: 0 }; // drives the animated churn on the Sun's surface

  private tmp = new Vector3();
  private tmp2 = new Vector3();
  private tmp3 = new Vector3();

  constructor(canvas: HTMLCanvasElement) {
    this.scale = getScale(this.state.scaleMode);

    this.renderer = new WebGLRenderer({ canvas, antialias: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.setClearColor(0x05060a, 1);

    this.camera = new PerspectiveCamera(50, 1, 0.001, 100000);
    this.camera.position.set(0, 70, 130);

    this.labelRenderer = new CSS2DRenderer();
    this.labelRenderer.domElement.className = 'label-layer';
    document.body.appendChild(this.labelRenderer.domElement);

    // Attach to the WebGL canvas, NOT the label layer (which is pointer-events:
    // none, so it would never receive drags).
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.enableZoom = true; // free flight: the user owns the camera
    // Track the pointer for free-explore hover highlighting.
    const cv = this.renderer.domElement;
    cv.addEventListener('pointermove', (e) => {
      const r = cv.getBoundingClientRect();
      this.pointerNDC.set(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1);
    });
    cv.addEventListener('pointerleave', () => { this.pointerNDC.set(2, 2); });

    // Drag to rotate: suspend auto-framing while dragging; on release, ease
    // back to the home framing or stay put (the default).
    this.controls.addEventListener('start', () => {
      this.userDragging = true;
      // Free explore + following: keep the pose the user is creating.
      if (this.followId && !this.returnOnRelease) this.followUserAdjusted = true;
    });
    // Wheel-zoom while following must not be undone by the follow framing.
    cv.addEventListener('wheel', () => {
      if (this.followId) this.followUserAdjusted = true;
    }, { passive: true });
    this.controls.addEventListener('end', () => {
      this.userDragging = false;
      if (this.returnOnRelease && !this.followId && this.homeCamPos) {
        this.camPosGoal = this.homeCamPos.clone();
        this.camTargetGoal.copy(this.homeCamTarget);
      } else if (!this.returnOnRelease) {
        this.camPosGoal = null; // free explore: keep the user's new view
      }
    });

    this.buildLights();
    this.buildStarfield();
    this.buildBodies();
    this.buildMoons();
    this.buildPortRings();
    this.buildNBody();
    this.navViz = new NavViz(this.scene);
    this.resize();
    window.addEventListener('resize', () => this.resize());
  }

  private buildLights(): void {
    const sunLight = new PointLight(0xfff2d0, 4, 0, 0.2);
    sunLight.position.set(0, 0, 0);
    this.scene.add(sunLight);
    this.scene.add(new AmbientLight(0x222a3a, 1.1));
  }

  /** Make the Sun's surface churn: warp the texture lookup with a time-varying
   *  ripple, and pulse the brightness a touch — so it boils rather than sits flat. */
  private animateSunSurface(mat: MeshBasicMaterial): void {
    mat.onBeforeCompile = (shader) => {
      shader.uniforms.uTime = this.sunTime;
      shader.fragmentShader = 'uniform float uTime;\n' + shader.fragmentShader.replace(
        '#include <map_fragment>',
        `#ifdef USE_MAP
          vec2 warpUv = vMapUv + 0.010 * vec2(
            sin(vMapUv.y * 26.0 + uTime * 1.2) + sin(vMapUv.x * 17.0 - uTime * 0.7),
            cos(vMapUv.x * 22.0 - uTime * 1.0) + sin(vMapUv.y * 13.0 + uTime * 0.5)
          );
          vec4 sampledDiffuseColor = texture2D( map, warpUv );
          float flicker = 1.0 + 0.06 * sin(uTime * 2.3 + vMapUv.x * 40.0);
          diffuseColor *= sampledDiffuseColor * flicker;
        #endif`,
      );
    };
    mat.needsUpdate = true;
  }

  private buildStarfield(): void {
    const N = 4000;
    const pos = new Float32Array(N * 3);
    let seed = 1337;
    const rand = () => {
      seed = (seed * 1103515245 + 12345) & 0x7fffffff;
      return seed / 0x7fffffff;
    };
    for (let i = 0; i < N; i++) {
      const r = 4000 + rand() * 4000;
      const theta = rand() * Math.PI * 2;
      const phi = Math.acos(2 * rand() - 1);
      pos[i * 3] = r * Math.sin(phi) * Math.cos(theta);
      pos[i * 3 + 1] = r * Math.cos(phi);
      pos[i * 3 + 2] = r * Math.sin(phi) * Math.sin(theta);
    }
    const geo = new BufferGeometry();
    geo.setAttribute('position', new Float32BufferAttribute(pos, 3));
    const mat = new PointsMaterial({ color: 0xffffff, size: 6, sizeAttenuation: true, transparent: true, opacity: 0.8 });
    this.scene.add(new Points(geo, mat));
  }

  private makeLabel(text: string, cls: string): CSS2DObject {
    const el = document.createElement('div');
    el.className = cls;
    el.textContent = text;
    return new CSS2DObject(el);
  }

  private buildBodies(): void {
    for (const body of ALL_BODIES) {
      const isStar = body.id === 'sun';
      const r = this.scale.bodyRadius(body.radius, isStar);
      const geo = new SphereGeometry(1, isStar ? 64 : 48, isStar ? 64 : 48);
      const tex = surfaceTexture(body.id, body.color);
      const mat = isStar
        ? new MeshBasicMaterial({ map: tex })
        : new MeshStandardMaterial({ map: tex, bumpMap: tex, bumpScale: 0.015, roughness: 0.92, metalness: 0.0 });
      if (isStar) this.animateSunSurface(mat as MeshBasicMaterial);
      const mesh = new Mesh(geo, mat);
      mesh.scale.setScalar(r);
      // Apply the axial tilt *outside* the daily spin (rotation.y) so the pole
      // stays fixed in space — spinning rotates the surface around a stationary
      // tilted axis, not precessing it (which only happens over ~26,000 yr).
      mesh.rotation.order = 'ZYX';
      mesh.rotation.z = MathUtils.degToRad(body.axialTilt);
      mesh.userData.id = body.id; // for hover raycasting
      this.scene.add(mesh);

      if (isStar) {
        // Two faint additive shells -> a soft halo that fades outward, rather
        // than one flat disk.
        for (const [s, o] of [[1.35, 0.22], [1.7, 0.1]] as const) {
          const glow = new Mesh(
            new SphereGeometry(1, 32, 32),
            new MeshBasicMaterial({ color: 0xffcf66, transparent: true, opacity: o, blending: AdditiveBlending, side: BackSide, depthWrite: false }),
          );
          glow.scale.setScalar(s);
          mesh.add(glow);
        }
      }
      if (body.id === 'saturn') {
        const ringGeo = new RingGeometry(1.35, 2.35, 96, 1);
        // Remap UVs so u runs radially (inner→outer), letting the ring profile
        // texture paint concentric bands and the Cassini gap.
        const pos = ringGeo.attributes.position;
        const uv = ringGeo.attributes.uv;
        for (let k = 0; k < pos.count; k++) {
          const rr = Math.hypot(pos.getX(k), pos.getY(k));
          uv.setXY(k, (rr - 1.35) / (2.35 - 1.35), 0.5);
        }
        const ring = new Mesh(
          ringGeo,
          new MeshBasicMaterial({ map: ringTexture(), side: DoubleSide, transparent: true, opacity: 0.9 }),
        );
        ring.rotation.x = Math.PI / 2;
        mesh.add(ring);
      }

      let orbitAU: Vector3[] = [];
      let orbitLine: Line | null = null;
      if (body.orbit) {
        orbitAU = sampleOrbit(body.orbit, 600);
        const lgeo = new BufferGeometry();
        lgeo.setAttribute('position', new Float32BufferAttribute(new Float32Array(orbitAU.length * 3), 3));
        orbitLine = new Line(lgeo, new LineBasicMaterial({ color: dim(body.color, 0.55), transparent: true, opacity: 0.6 }));
        this.scene.add(orbitLine);
      }

      const label = this.makeLabel(body.name, 'body-label');
      mesh.add(label);

      this.views.push({ body, mesh, orbitAU, orbitLine, label, spin: 0, opacity: 1, curAU: new Vector3(), curScene: new Vector3() });
    }
  }

  private buildMoons(): void {
    for (const planet of PLANETS) {
      if (!planet.moons?.length) continue;
      let minA = Infinity;
      for (const moon of planet.moons) minA = Math.min(minA, moon.aKm / AU_KM);
      this.minMoonA.set(planet.id, minA);

      for (const moon of planet.moons) {
        const mtex = surfaceTexture(moon.id, moon.color);
        const mesh = new Mesh(
          new SphereGeometry(1, 28, 28),
          new MeshStandardMaterial({ map: mtex, bumpMap: mtex, bumpScale: 0.01, roughness: 0.95 }),
        );
        this.scene.add(mesh);

        // Sample one full relative orbit (ecliptic AU about the planet).
        const mu = pairMu(planet, moon);
        const el = moonElements(moon);
        const period = (2 * Math.PI) / Math.sqrt(mu / Math.pow(el.a * 1.495978707e11, 3)) / DAY;
        const orbitRelAU: Vector3[] = [];
        const segs = 256;
        for (let k = 0; k <= segs; k++) {
          orbitRelAU.push(keplerPosition(el, (k / segs) * period, mu));
        }
        const lgeo = new BufferGeometry();
        lgeo.setAttribute('position', new Float32BufferAttribute(new Float32Array(orbitRelAU.length * 3), 3));
        const orbitLine = new Line(lgeo, new LineBasicMaterial({ color: dim(moon.color, 0.6), transparent: true, opacity: 0.45 }));
        this.scene.add(orbitLine);

        const label = this.makeLabel(moon.name, 'moon-label');
        mesh.add(label);

        this.moonViews.push({ moon, parent: planet, mesh, orbitRelAU, orbitLine, label, spin: 0, opacity: 0 });
      }
    }
  }

  /** (Re)build the N-body integrator and seed it from the current sim time. */
  private buildNBody(): void {
    this.simBodies = buildSimBodies(this.state.showMoons);
    this.simIndexByPlanet.clear();
    this.simBodies.forEach((d, i) => {
      if (d.kind !== 'moon') this.simIndexByPlanet.set(d.id, i);
    });
    this.nbody = new NBody(this.simBodies.map((d) => d.mass));
    this.seedNBody();
  }

  private seedNBody(): void {
    const pos: Vector3[] = [];
    const vel: Vector3[] = [];
    for (const d of this.simBodies) {
      const s = descriptorState(d, this.simDays);
      pos.push(s.pos);
      vel.push(s.vel);
    }
    this.nbody.seed(pos, vel);
    this.energy0 = this.nbody.totalEnergy();
  }

  setScaleMode(mode: ScaleMode): void {
    this.state.scaleMode = mode;
    this.scale = getScale(mode);
    for (const v of this.views) {
      v.mesh.scale.setScalar(this.scale.bodyRadius(v.body.radius, v.body.id === 'sun'));
      const m = v.mesh.material as MeshStandardMaterial;
      if (m.emissive) {
        m.emissive.setHex(0x000000);
        m.emissiveIntensity = 1;
        if (m.emissiveMap) { m.emissiveMap = null; m.needsUpdate = true; }
      }
    }
    for (const mv of this.moonViews) {
      mv.mesh.scale.setScalar(this.scale.bodyRadius(mv.moon.radius, false));
    }
    this.rebuildOrbits();
  }

  setPhysics(mode: PhysicsMode): void {
    this.state.physics = mode;
    if (mode === 'nbody') this.seedNBody();
  }

  setShowMoons(on: boolean): void {
    this.state.showMoons = on;
    // Moon gravity changes the N-body body set, so rebuild it.
    this.buildNBody();
  }

  setTwoD(on: boolean): void {
    this.state.twoD = on ? 1 : 0;
  }

  /** Step 5: continue the drifting Earth, then bend it into an orbit as gravity
   *  (the Sun) and the vectors fade in. A small fixed-Sun 2-body sim. */

  /**
   * 太空港示意环。真实同步轨道半径（地球 42,164 km）在行星尺度场景里远小于
   * 被夸大的天体半径，画不出来 —— 这里按"1.7 × 天体显示半径"画一个示意环，
   * 明确标注（示意），只表达"港口在天体周围而不是天体中心"这件事。
   */
  private buildPortRings(): void {
    for (let i = 0; i < 2; i++) {
      const pts: Vector3[] = [];
      for (let k = 0; k <= 96; k++) {
        const a = (2 * Math.PI * k) / 96;
        pts.push(new Vector3(Math.cos(a), 0, Math.sin(a)));
      }
      const ring = new Line(
        new BufferGeometry().setFromPoints(pts),
        new LineBasicMaterial({ color: 0x6ee7ff, transparent: true, opacity: 0.55 }),
      );
      ring.visible = false;
      ring.frustumCulled = false;
      this.scene.add(ring);
      const label = this.makeLabel('', 'vec-label');
      label.visible = false;
      this.portRings.push({ ring, label });
    }
  }

  /** Place/refresh the two port rings for the current mission. */
  private updatePortRings(): void {
    const plan = this.mission?.plan;
    const ids = [plan?.departureId, plan?.targetId];
    const ports = [plan?.departPort, plan?.arrivePort];
    for (let i = 0; i < this.portRings.length; i++) {
      const { ring, label } = this.portRings[i];
      const view = ids[i] ? this.views.find((x) => x.body.id === ids[i]) : undefined;
      const port = ports[i];
      if (!plan || !port || !view) {
        ring.visible = false;
        label.visible = false;
        continue;
      }
      this.bodyScenePos(view.body.id, this.portTmp);
      ring.position.copy(this.portTmp);
      ring.scale.setScalar(Math.max(view.mesh.scale.x * 1.7, 0.12));
      ring.visible = true;
      label.position.copy(this.portTmp);
      label.visible = true;
      label.element.textContent =
        `${i === 0 ? '出发港' : '目标港'} · ${port.kind === 'synchronous' ? '同步' : '停泊'}轨道`
        + ` ${Math.round(port.altitudeKm).toLocaleString()} km（示意）`;
    }
  }

  /** Whether the camera springs back to its home framing after a drag. */
  setCameraReturn(on: boolean): void { this.returnOnRelease = on; }

  /** Enable mouse-wheel zoom. */
  setZoomEnabled(on: boolean): void { this.controls.enableZoom = on; }

  /** Queue a smooth camera fly-to (eased each frame in update()). */
  private flyTo(pos: Vector3, target: Vector3): void {
    this.followId = null;
    this.camPosGoal = pos.clone();
    this.camTargetGoal.copy(target);
    this.homeCamPos = pos.clone();      // remembered for spring-back after a drag
    this.homeCamTarget.copy(target);
  }

  /**
   * Keep the camera framed on a body as it moves (2D overhead). Used for the
   * Earth–Moon slides, where Earth orbits the Sun so a fixed camera would lose
   * it. The view eases toward the moving target each frame.
   */
  followBody(id: string, distanceMul = 10, raiseFactor = 0.34, sideView = false): void {
    const v = this.views.find((x) => x.body.id === id);
    const radius = v ? v.mesh.scale.x : 1;
    const dist = Math.max(radius * distanceMul, 8);
    if (sideView) {
      // 3/4 view from in front (+Z) and slightly above, so an axial tilt reads
      // as a lean — used for the self-rotation slide (which would otherwise show
      // the tilt foreshortened from straight overhead).
      this.followCamOffset.set(0, dist * 0.42, dist * 0.92);
      this.followTgtOffset.set(0, 0, 0);
    } else {
      const raise = dist * raiseFactor; // 0 = body dead-center on screen
      this.followCamOffset.set(0, dist, raise + 0.001); // tiny z avoids gimbal at raise=0
      this.followTgtOffset.set(0, 0, raise);
    }
    this.followId = id;
    this.followHasLast = false;
    this.followUserAdjusted = false; // a fresh follow re-applies its framing
    this.homeCamPos = null; // home is the (dynamic) follow pose
  }

  stopFollow(): void {
    this.followId = null;
    this.followUserAdjusted = false;
  }

  /**
   * Start flying a planned transfer. The craft waits docked at the departure
   * body until the departure epoch, cruises along the Lambert arc, then rides
   * with the target body after arrival. With `injectError` a small launch
   * dispersion is added so the flight drifts off the plan — the setup for
   * trajectory correction manoeuvres.
   */
  launchMission(
    plan: TransferPlan,
    opts: { injectError?: boolean; tracking?: TrackingTier | null } = {},
  ): void {
    // 进入任务视角：确保是可自由拖拽/滚轮缩放的状态（导览模式的"回弹+禁缩放"
    // 会一路带进来，让用户觉得视角被锁死）。
    this.setCameraReturn(false);
    this.setZoomEnabled(true);
    const mission = new FlightMission(plan, {
      injectError: opts.injectError ?? true,
      tracking: opts.tracking ?? null,
    });
    this.missionTrackingLabel = opts.tracking?.label ?? null;
    this.mission = mission;
    this.missionPhase = this.simDays < plan.departureDay ? 'docked' : 'cruise';
    const f = 1 - this.flatten;
    this.navViz.setPlanArc(mission.planArcPath, plan.r1, plan.r2, this.scale, f);
    this.navViz.setPredicted(mission.predictedPath(), this.scale, f);
    this.adcs = null; // created when the cruise starts
    const name = ALL_BODIES.find((b) => b.id === plan.targetId)?.name ?? plan.targetId;
    this.navViz.setLabel(`飞船 → ${name}`);
    this.updateMission();
  }

  /**
   * 手动点火：在当前时刻沿指定方向给飞船一个 Δv（km/s）。
   * 方向取飞船当前的真值状态：顺行/逆行（沿速度）、径向外/内（沿日心矢径）、
   * 法向/反法向（沿轨道面法线）。点火即写入真值轨迹（与 TCM 同一套机制），
   * 因此会真实改变后续弹道——可以自己"开"飞船。
   */
  manualBurn(dvKms: number, dir: ManualBurnDir): number | null {
    if (!this.mission || dvKms === 0) return null;
    const st = this.mission.stateAt(this.simDays);
    const fwd = st.vel.clone().normalize();
    let axis = fwd;
    if (dir === 'retrograde') axis = fwd.clone().negate();
    else if (dir !== 'prograde') {
      const rHat = st.pos.clone().normalize();
      if (dir === 'radialOut') axis = rHat;
      else if (dir === 'radialIn') axis = rHat.clone().negate();
      else {
        const h = new Vector3().crossVectors(st.pos, st.vel).normalize();
        axis = dir === 'normal' ? h : h.negate();
      }
    }
    return this.mission.applyManualBurn(this.simDays, axis.multiplyScalar(fromKms(dvKms)));
  }

  /**
   * 油门：设定推进系统与连续推力（level 0..1，dir = 推力方向）。
   * 与脉冲式手动点火互补——化学/核热用脉冲，核电这类低推力靠持续点火。
   */
  setThrottle(level: number, dir: ThrustDir, driveId: string): void {
    if (!this.mission) return;
    this.mission.setThrottle(this.simDays, driveId, Math.min(1, Math.max(0, level)), dir);
  }

  /** Remove the spacecraft and its trajectory. */
  clearMission(): void {
    this.mission = null;
    this.navViz.clear();
    for (const { ring, label } of this.portRings) {
      ring.visible = false;
      label.visible = false;
    }
    if (this.followId === CRAFT_ID) this.followId = null;
  }

  /**
   * Execute a trajectory correction manoeuvre right now: re-solve Lambert from
   * the craft's actual state to the rendezvous point and fly the new arc.
   * Returns the applied delta-v in km/s, or null when unavailable.
   */
  applyTcm(): number | null {
    const m = this.mission;
    if (!m || this.missionPhase !== 'cruise') return null;
    const dvKms = m.applyTcm(this.simDays);
    if (dvKms === null) return null;
    this.navViz.setPredicted(m.predictedPath(), this.scale, 1 - this.flatten);
    this.updateMission();
    return dvKms;
  }

  /** Snapshot for the dedicated navigation console (null without a mission). */
  navSnapshot(): NavSnapshot | null {
    const m = this.mission;
    if (!m) return null;
    const status = this.missionStatus();
    if (!status) return null;
    const p = m.plan;
    const t = this.simDays;
    const findView = (id: string) => this.views.find((v) => v.body.id === id);
    const bodyVel = (id: string): Vector3 => {
      const b = ALL_BODIES.find((x) => x.id === id);
      return b?.orbit ? keplerState(b.orbit, t).vel : new Vector3();
    };

    const targetAU = findView(p.targetId)?.curAU.clone() ?? p.r2.clone();
    let shipAU: Vector3;
    let shipVel: Vector3;
    if (t < p.departureDay || t >= p.arrivalDay) {
      const id = t < p.departureDay ? p.departureId : p.targetId;
      shipAU = findView(id)?.curAU.clone() ?? p.r1.clone();
      shipVel = bodyVel(id);
    } else {
      const st = m.stateAt(t);
      shipAU = st.pos;
      shipVel = st.vel;
    }

    const rel = new Vector3().subVectors(targetAU, shipAU);
    const dist = rel.length();
    const closing = dist > 0
      ? rel.dot(new Vector3().subVectors(shipVel, bodyVel(p.targetId))) / dist
      : 0;

    // --- attitude: nose held prograde, "up" is the ecliptic-north component.
    const forward = shipVel.clone().normalize();
    const north = new Vector3(0, 0, 1);
    const up = new Vector3().copy(north).addScaledVector(forward, -north.dot(forward));
    if (up.lengthSq() < 1e-8) up.set(1, 0, 0).addScaledVector(forward, -forward.x);
    up.normalize();
    const right = new Vector3().crossVectors(up, forward).normalize();

    const toCraft = (dir: Vector3): PointingReading => {
      const d = dir.clone().normalize();
      const fwd = Math.max(-1, Math.min(1, d.dot(forward)));
      return {
        azDeg: (Math.atan2(d.dot(right), fwd) * 180) / Math.PI,
        elDeg: (Math.asin(Math.max(-1, Math.min(1, d.dot(up)))) * 180) / Math.PI,
        offDeg: (Math.acos(fwd) * 180) / Math.PI,
        posDeg: (Math.atan2(d.dot(up), d.dot(right)) * 180) / Math.PI,
      };
    };

    const sunDir = shipAU.clone().multiplyScalar(-1).normalize();
    const earthAU = findView('earth')?.curAU.clone() ?? new Vector3(1, 0, 0);
    const sun = toCraft(sunDir);
    const target = toCraft(new Vector3().subVectors(targetAU, shipAU));
    const earth = toCraft(new Vector3().subVectors(earthAU, shipAU));

    // --- accelerations: modelled gravity (what shapes the orbit) and the
    // accelerometer reading (non-gravitational; coasting here, so noise only).
    // Parked on a body (or within ~15,000 km of one) the point-mass model is
    // singular, so the readout blanks instead of diverging.
    if (!this.navForceModel || !this.navSources) {
      this.navSources = solarSystemSources();
      this.navForceModel = makeForceModel(this.navSources);
    }
    let nearestBody = Infinity;
    for (const src of this.navSources) {
      const d = src.positionAt(t).distanceTo(shipAU);
      if (d < nearestBody) nearestBody = d;
    }
    const accelOk = this.missionPhase === 'cruise' && nearestBody > 1e-4;

    const g = accelOk ? this.navForceModel(t, shipAU, new Vector3()) : null;
    const gravityMms2 =
      g && Number.isFinite(g.length()) && g.length() < 1
        ? ((g.length() * AU) / (DAY * DAY)) * 1000
        : null;
    const gravity = g && gravityMms2 !== null ? toCraft(g) : null;
    // 非引力加速度 = 太阳光压（真值模型里唯一未建模为引力的力）：a = P·Cr·A/m / r²，
    // 单位 µg（1 g = 9.80665 m/s²）。不再用伪造读数。
    const rHelioAu = Math.max(shipAU.length(), 1e-6);
    const srpMps2 =
      (P_SRP_1AU * DEFAULT_SRP.cr * DEFAULT_SRP.areaM2) / DEFAULT_SRP.massKg / (rHelioAu * rHelioAu);
    const nonGravUg = accelOk ? (srpMps2 / 9.80665) * 1e6 : 0;

    const orbit = elementsFromState({ pos: shipAU, vel: shipVel });
    const gyroDegPerDay =
      g && gravityMms2 !== null && shipVel.lengthSq() > 0
        ? (new Vector3().crossVectors(shipVel, g).length() / shipVel.lengthSq()) * (180 / Math.PI)
        : 0;

    const raw = m.lastMeasurement();
    const tn = m.trackingNoise();
    const measurement = raw.range
      ? {
          t: raw.range.t,
          rangeKm: (raw.range.range * AU) / 1000,
          rangeRateKms: toKms(raw.range.rangeRate),
          opticalOffDeg: raw.optical ? toCraft(raw.optical.dir).offDeg : null,
          rangeSigmaKm: tn ? (tn.rangeSigma * AU) / 1000 : null,
          rateSigmaKms: tn ? toKms(tn.rangeRateSigma) : null,
          opticalSigmaArcsec: tn ? (tn.opticalSigma * 180 * 3600) / Math.PI : null,
        }
      : null;

    return {
      phase: this.missionPhase,
      departureId: p.departureId,
      targetId: p.targetId,
      simDays: t,
      departureDay: p.departureDay,
      arrivalDay: p.arrivalDay,
      tof: p.tof,
      progress: Math.min(1, Math.max(0, (t - p.departureDay) / p.tof)),
      shipAU,
      shipSpeedKms: toKms(shipVel.length()),
      escapeSpeedKms: Math.sqrt((2 * MU_SUN) / Math.max(shipAU.length(), 1e-9)) * AUDAY_TO_KMS,
      escapeFraction:
        shipVel.length() / Math.sqrt((2 * MU_SUN) / Math.max(shipAU.length(), 1e-9)),
      targetAU,
      targetDistKm: (dist * AU) / 1000,
      closingKms: toKms(closing),
      planArc: m.planArcPath,
      flown: m.flown,
      predicted: m.predictedPath(),
      status,
      adcs: this.adcs
        ? {
            pointingArcsec: this.adcsPointingArcsec,
            estimateArcsec: this.adcs.estimateErrorArcsec(),
            rateDegS: this.adcs.rateDegPerSec(),
            starUpdates: this.adcs.starUpdates,
            starSigmaArcsec: DEFAULT_ADCS.starSigmaArcsec,
            gyroArwDegSqrtH: DEFAULT_ADCS.gyroArwDegSqrtH,
          }
        : null,
      attitude: {
        yawDeg: (Math.atan2(forward.y, forward.x) * 180) / Math.PI,
        pitchDeg: (Math.asin(Math.max(-1, Math.min(1, forward.z))) * 180) / Math.PI,
        gyroDegPerDay,
        sun,
        target,
        earth,
      },
      accel: { gravityMms2, gravity, nonGravUg },
      orbit,
      measurement,
    };
  }

  /** Keep the camera on the spacecraft (uses the generic follow machinery). */
  followCraft(): void {
    if (this.mission) this.followBody(CRAFT_ID, 14, 0.3);
  }

  /** Snapshot of the mission for the UI, or null when there is no mission. */
  missionStatus(): MissionStatus | null {
    if (!this.mission) return null;
    const m = this.mission;
    const p = m.plan;
    const sol = this.missionPhase === 'cruise' ? m.solveTcm(this.simDays) : null;
    return {
      departureId: p.departureId,
      targetId: p.targetId,
      phase: this.missionPhase,
      departureDay: p.departureDay,
      arrivalDay: p.arrivalDay,
      tof: p.tof,
      dvDepart: p.dvDepart,
      dvArrive: p.dvArrive,
      dvTotal: p.dvTotal,
      vinfDepartKms: p.vinfDepartKms,
      vinfArriveKms: p.vinfArriveKms,
      departPort: p.departPort,
      arrivePort: p.arrivePort,
      daysToDeparture: p.departureDay - this.simDays,
      daysToArrival: p.arrivalDay - this.simDays,
      missKm: (m.truthMissDistance() * AU) / 1000,
      estMissKm: (m.estimatedMissDistance(this.simDays) * AU) / 1000,
      tcmDvKms: sol ? sol.dvKms : null,
      tcmCount: m.tcmCount,
      tcmUsedKms: m.tcmUsedKms,
      manualCount: m.manualCount,
      manualUsedKms: m.manualUsedKms,
      driveLabel: m.throttleState()?.drive.label ?? '化学',
      massKg: m.massKg(this.simDays),
      propellantKg: m.massKg(this.simDays) - 900,
      thrustMms2: m.thrustAccelMps2(this.simDays) * 1000,
      thrustActive: m.throttleState()?.active ?? false,
      throttleLevel: m.throttleState()?.level ?? 0,
      trackingLabel: this.missionTrackingLabel,
      trackCount: m.trackingCount,
      estErrorKm: m.hasTracking ? (m.estimateError(this.simDays) * AU) / 1000 : 0,
      posSigmaKm: m.hasTracking ? (m.positionSigma() * AU) / 1000 : 0,
    };
  }

  /** Per-frame mission update: dock → cruise → arrived, with TCM support. */
  private updateMission(): void {
    this.updatePortRings();
    const m = this.mission;
    if (!m) return;
    const p = m.plan;
    const f = 1 - this.flatten;
    const t = this.simDays;

    if (t < p.departureDay) {
      this.missionPhase = 'docked';
      const dv = this.views.find((v) => v.body.id === p.departureId);
      this.navViz.updateCraft(dv ? dv.curAU : p.r1, null, this.scale, f);
      const r = dv ? this.scale.bodyRadius(dv.body.radius, false) : 0.4;
      this.navViz.setCraftHover(r + 0.55);
      this.navViz.setFlown([], this.scale, f);
      this.navViz.setPredicted(null, this.scale, f);
    } else if (t < p.arrivalDay) {
      this.missionPhase = 'cruise';
      m.updateFlown(t);
      const st = m.stateAt(t);
      // ADCS: hold the nose prograde with a PD loop; gyro + star tracker keep
      // the onboard attitude estimate (the flight-deck readouts).
      const north = new Vector3(0, 0, 1);
      const fwd = st.vel.clone().normalize();
      const upCmd = north.clone().addScaledVector(fwd, -north.dot(fwd));
      if (upCmd.lengthSq() < 1e-8) upCmd.set(0, 1, 0).addScaledVector(fwd, -fwd.y);
      upCmd.normalize();
      if (!this.adcs) {
        this.adcs = new Adcs(fwd, upCmd, t);
        this.adcsDay = t;
      }
      const dtAtt = t - this.adcsDay;
      if (dtAtt > 0) {
        this.adcs.advance(dtAtt, fwd, upCmd);
        this.adcsDay = t;
      }
      this.adcsPointingArcsec = this.adcs.pointingErrorDeg(fwd) * 3600;
      this.navViz.updateCraft(st.pos, st.vel, this.scale, f);
      this.navViz.setFlown(m.flown, this.scale, f);
      this.navViz.setPredicted(m.predictedPath(), this.scale, f);
    } else {
      this.missionPhase = 'arrived';
      m.updateFlown(p.arrivalDay);
      const tv = this.views.find((v) => v.body.id === p.targetId);
      this.navViz.updateCraft(tv ? tv.curAU : p.r2, null, this.scale, f);
      const r = tv ? this.scale.bodyRadius(tv.body.radius, false) : 0.4;
      this.navViz.setCraftHover(r + 0.55);
      this.navViz.setFlown(m.flown, this.scale, f);
    }
  }

  /**
   * The body's intended scene position for the CURRENT step's state (physics +
   * target flatten) — computed fresh rather than read from the mesh, whose
   * position still reflects the previous step.
   */
  private bodyScenePos(id: string, out: Vector3): Vector3 {
    const v = this.views.find((x) => x.body.id === id);
    if (!v) return out.set(0, 0, 0);
    if (v.body.orbit) {
      if (this.state.physics === 'kepler') this.tmp.copy(keplerPosition(v.body.orbit, this.simDays));
      else this.nbody.positionAU(this.simIndexByPlanet.get(id)!, this.tmp);
    } else {
      this.tmp.set(0, 0, 0);
    }
    this.scale.position(this.tmp, this.tmp);
    eclToScene(this.tmp, out);
    out.y *= 1 - this.state.twoD; // target flatten for the new step
    return out;
  }

  focusOn(id: string, distanceMul = 6): void {
    const v = this.views.find((x) => x.body.id === id);
    const target = this.bodyScenePos(id, new Vector3());
    const radius = v ? v.mesh.scale.x : 3;
    const dist = Math.max(radius * distanceMul, 8);

    if (this.state.twoD) {
      // Overhead for the 2D ecliptic view; shift the aim so the body sits in
      // the upper area (clear of the bottom panels). Screen-up is −Z.
      const raise = dist * 0.4;
      const aim = new Vector3(target.x, 0, target.z + raise);
      this.flyTo(new Vector3(target.x, dist, target.z + raise + 0.001), aim);
      return;
    }

    // 3D: view from the sunlit side (camera between Sun and body) with a little
    // elevation; the body is centered on screen (target = body).
    const sunward = target.lengthSq() > 1e-6 ? target.clone().multiplyScalar(-1).normalize() : new Vector3(0, 0, 1);
    const dir = sunward.add(new Vector3(0, 0.5, 0)).normalize();
    const goalPos = target.clone().add(dir.clone().multiplyScalar(dist));
    this.flyTo(goalPos, target);
  }

  private moonFactor(parentId: string, parentRenderRadius: number): number {
    if (this.scale.mode === 'real') return TRUE_UNITS_PER_AU;
    const minA = this.minMoonA.get(parentId) ?? 0.001;
    return (parentRenderRadius * 1.9) / minA; // innermost moon sits ~1.9 radii out
  }

  private rebuildOrbits(): void {
    const f = 1 - this.flatten;
    for (const v of this.views) {
      if (!v.orbitLine) continue;
      const attr = (v.orbitLine.geometry as BufferGeometry).getAttribute('position');
      const arr = attr.array as Float32Array;
      for (let k = 0; k < v.orbitAU.length; k++) {
        this.scale.position(v.orbitAU[k], this.tmp);
        eclToScene(this.tmp, this.tmp2);
        this.tmp2.y *= f;
        arr[k * 3] = this.tmp2.x; arr[k * 3 + 1] = this.tmp2.y; arr[k * 3 + 2] = this.tmp2.z;
      }
      attr.needsUpdate = true;
    }
    for (const mv of this.moonViews) {
      const factor = this.moonFactor(mv.parent.id, this.scale.bodyRadius(mv.parent.radius, false));
      const attr = (mv.orbitLine.geometry as BufferGeometry).getAttribute('position');
      const arr = attr.array as Float32Array;
      for (let k = 0; k < mv.orbitRelAU.length; k++) {
        this.tmp.copy(mv.orbitRelAU[k]).multiplyScalar(factor);
        eclToScene(this.tmp, this.tmp2);
        this.tmp2.y *= f;
        arr[k * 3] = this.tmp2.x; arr[k * 3 + 1] = this.tmp2.y; arr[k * 3 + 2] = this.tmp2.z;
      }
      attr.needsUpdate = true;
    }
    if (this.navViz) this.navViz.project(this.scale, f);
  }

  private isVisible(id: string): boolean {
    return this.visible ? this.visible.has(id) : true;
  }

  update(dtReal: number): void {
    const s = this.state;
    this.sunTime.value += dtReal; // animate the Sun's surface

    if (!s.paused) {
      const dtDays = dtReal * s.daysPerSecond;
      this.simDays += dtDays;
      if (s.physics === 'nbody') this.stepNBody(dtDays);
    }

    const prevFlatten = this.flatten;
    this.flatten = MathUtils.damp(this.flatten, s.twoD, 4, dtReal);
    const flattenMoving = Math.abs(this.flatten - prevFlatten) > 1e-4;

    const targetPolar = s.twoD ? 0.0001 : Math.PI;
    if (s.twoD && this.userDragging) {
      // While tilting a 2D slide, hold polarLimit at the *live* tilt and free the
      // angle. That way, when the lock is re-applied on release, it eases back to
      // flat from where the user left it instead of snapping (it would otherwise
      // have kept damping to flat in the background during the drag).
      this.tmp.copy(this.camera.position).sub(this.controls.target);
      this.polarLimit = Math.acos(MathUtils.clamp(this.tmp.y / (this.tmp.length() || 1), -1, 1));
      this.controls.minPolarAngle = 0;
      this.controls.maxPolarAngle = Math.PI;
    } else {
      // Lock to top-down for 2D (easing there); free for 3D.
      this.polarLimit = MathUtils.damp(this.polarLimit, targetPolar, 4, dtReal);
      if (s.twoD) {
        this.controls.minPolarAngle = this.polarLimit;
        this.controls.maxPolarAngle = this.polarLimit;
      } else {
        this.controls.minPolarAngle = 0;
        this.controls.maxPolarAngle = Math.PI;
      }
    }

    const f = 1 - this.flatten;

    // Free-explore hover: which body is under the pointer (reveals its label/orbit).
    if (this.hoverEnabled) {
      this.raycaster.setFromCamera(this.pointerNDC, this.camera);
      const meshes = this.views.filter((v) => v.mesh.visible).map((v) => v.mesh);
      const hits = this.raycaster.intersectObjects(meshes, false);
      this.hoveredId = hits.length ? (hits[0].object.userData.id as string) : null;
    } else {
      this.hoveredId = null;
    }

    // Planets + Sun.
    for (let idx = 0; idx < this.views.length; idx++) {
      const v = this.views[idx];
      const shown = this.isVisible(v.body.id);
      v.opacity = MathUtils.damp(v.opacity, shown ? 1 : 0, 6, dtReal);
      const vis = v.opacity > 0.02;
      v.mesh.visible = vis;
      const mat = v.mesh.material as MeshStandardMaterial;
      mat.transparent = v.opacity < 0.995;
      mat.opacity = v.opacity;

      if (v.body.orbit) {
        if (s.physics === 'kepler') {
          this.tmp.copy(keplerPosition(v.body.orbit, this.simDays));
        } else {
          this.nbody.positionAU(this.simIndexByPlanet.get(v.body.id)!, this.tmp);
        }
      } else {
        if (s.physics === 'nbody') this.nbody.positionAU(this.simIndexByPlanet.get(v.body.id)!, this.tmp);
        else this.tmp.set(0, 0, 0);
      }
      v.curAU.copy(this.tmp);
      this.scale.position(this.tmp, this.tmp);
      eclToScene(this.tmp, this.tmp2);
      this.tmp2.y *= f;

      v.mesh.position.copy(this.tmp2);
      v.curScene.copy(this.tmp2);

      if (!s.paused && s.showSpin && v.body.rotationPeriod) {
        const rate = (2 * Math.PI) / Math.abs(v.body.rotationPeriod);
        v.spin += rate * dtReal * s.daysPerSecond * Math.sign(v.body.rotationPeriod);
        v.mesh.rotation.y = v.spin;
      }

      const hov = this.hoveredId === v.body.id;
      if (v.orbitLine) {
        v.orbitLine.visible = (s.showOrbits || hov) && vis;
        (v.orbitLine.material as LineBasicMaterial).opacity = 0.6 * v.opacity;
      }
      v.label.visible = (s.showLabels || hov) && v.opacity > 0.4;
      (v.label.element as HTMLElement).style.opacity = String(v.opacity);
    }

    // Moons (always Keplerian-rendered, relative to their planet).
    for (const mv of this.moonViews) {
      const parentView = this.views.find((x) => x.body.id === mv.parent.id)!;
      const moonShown = this.moonShown(mv);
      mv.opacity = MathUtils.damp(mv.opacity, moonShown ? 1 : 0, 6, dtReal);
      const mvis = mv.opacity > 0.02;
      mv.mesh.visible = mvis;
      (mv.mesh.material as MeshStandardMaterial).transparent = mv.opacity < 0.995;
      (mv.mesh.material as MeshStandardMaterial).opacity = mv.opacity;
      mv.orbitLine.visible = mvis && s.showOrbits;
      (mv.orbitLine.material as LineBasicMaterial).opacity = 0.45 * mv.opacity;
      mv.label.visible = mvis && s.showLabels && s.showMoonLabels && mv.opacity > 0.4;
      (mv.label.element as HTMLElement).style.opacity = String(mv.opacity);
      if (!mvis) continue;

      const factor = this.moonFactor(mv.parent.id, parentView.mesh.scale.x);
      this.tmp.copy(moonRelativePosition(mv.parent, mv.moon, this.simDays)).multiplyScalar(factor);
      eclToScene(this.tmp, this.tmp3);
      this.tmp3.y *= f;
      mv.mesh.position.copy(parentView.curScene).add(this.tmp3);
      mv.orbitLine.position.copy(parentView.curScene);

      // These moons are tidally locked: their rotation period equals their
      // orbital period, so the same hemisphere always faces the planet. Rather
      // than free-spin on an axis, orient a fixed meridian toward the parent —
      // a geometric truth that holds even when paused. (this.tmp3 is the
      // parent→moon offset, so the parent lies in the -tmp3 direction.)
      mv.mesh.rotation.y = Math.atan2(-this.tmp3.x, -this.tmp3.z);

    }

    if (flattenMoving) this.rebuildOrbits();

    this.updateMission();

    // Following a moving body — or the spacecraft. The followed scene position
    // is already fresh from this frame's updates above.
    if (this.followId) {
      let followScene: Vector3 | null = null;
      if (this.followId === CRAFT_ID) {
        if (this.mission) followScene = this.navViz.craftScene;
      } else {
        const fv = this.views.find((v) => v.body.id === this.followId);
        if (fv) followScene = fv.curScene;
      }
      if (followScene) {
        // 2D ecliptic view: hold the camera straight overhead so the flattened
        // scene reads as a clean plane (a tilted follow pose would skew it).
        if (this.state.twoD > 0.5) {
          const dist = this.followCamOffset.length() || 14;
          this.followCamOffset.set(0, Math.max(dist, 8), 0.001);
          this.followTgtOffset.set(0, 0, 0);
        }
        if (!this.followHasLast) { this.followLast.copy(followScene); this.followHasLast = true; }
        if (this.userDragging || this.followUserAdjusted) {
          // Rigid translation: carry the camera AND its pivot with the target,
          // so the user's own zoom/orbit pose is preserved while following.
          this.followDelta.copy(followScene).sub(this.followLast);
          this.camera.position.add(this.followDelta);
          this.controls.target.add(this.followDelta);
          this.camPosGoal = null;
        } else {
          this.camPosGoal = this.followCamPos.copy(followScene).add(this.followCamOffset);
          this.camTargetGoal.copy(followScene).add(this.followTgtOffset);
        }
        this.followLast.copy(followScene);
      }
    }

    // Ease the camera toward its goal (set by focusOn / frameRadius / follow).
    if (this.camPosGoal && !this.userDragging) {
      const k = 1 - Math.exp(-3.2 * dtReal);
      this.camera.position.lerp(this.camPosGoal, k);
      this.controls.target.lerp(this.camTargetGoal, k);
      if (!this.followId &&
          this.camera.position.distanceTo(this.camPosGoal) < 0.04 &&
          this.controls.target.distanceTo(this.camTargetGoal) < 0.04) {
        this.camera.position.copy(this.camPosGoal);
        this.controls.target.copy(this.camTargetGoal);
        this.camPosGoal = null;
      }
    }

    // Auto-rotate only once the framing has settled and the user isn't dragging,
    // so it doesn't fight the fly-in ease.
    this.controls.autoRotate = this.wantAutoRotate && !this.camPosGoal && !this.userDragging;

    this.controls.update();
    this.renderer.render(this.scene, this.camera);
    this.labelRenderer.render(this.scene, this.camera);
  }

  private moonShown(mv: MoonView): boolean {
    if (!this.isVisible(mv.parent.id)) return false;
    // A moon shows when its id is explicitly listed; otherwise
    // it follows the global "show moons" toggle.
    if (this.visible) return this.visible.has(mv.moon.id);
    return this.state.showMoons;
  }

  /** Integrate N-body across one frame; substep small enough for fast moons. */
  private stepNBody(dtDays: number): void {
    if (dtDays === 0) return;
    let maxStepDays = 0.5;
    if (this.state.showMoons) {
      const shortest = shortestMoonPeriod(this.simBodies);
      if (isFinite(shortest)) maxStepDays = Math.min(maxStepDays, shortest / 40);
    }
    let n = Math.ceil(Math.abs(dtDays) / maxStepDays);
    n = Math.min(n, 6000); // hard cap; accuracy degrades gracefully past here
    const stepSec = (dtDays / n) * DAY;
    for (let i = 0; i < n; i++) this.nbody.step(stepSec);
  }

  private resize(): void {
    const w = window.innerWidth, h = window.innerHeight;
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(w, h);
    this.labelRenderer.setSize(w, h);
  }
}

function dim(hex: ColorRepresentation, fac: number): number {
  const c = new Color(hex);
  c.multiplyScalar(fac);
  return c.getHex();
}

function smoothstep(t: number): number {
  const x = t < 0 ? 0 : t > 1 ? 1 : t;
  return x * x * (3 - 2 * x);
}

function clamp01(x: number): number { return x < 0 ? 0 : x > 1 ? 1 : x; }

const WHITE = new Color(0xffffff);
const UP_Y = new Vector3(0, 1, 0);
const ZERO = new Vector3(0, 0, 0);


export { orbitalPeriodDays };
