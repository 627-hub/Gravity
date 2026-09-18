import {
  BoxGeometry, BufferGeometry, ConeGeometry, CylinderGeometry,
  Float32BufferAttribute, Group, Line, LineBasicMaterial, Mesh, MeshBasicMaterial,
  MeshStandardMaterial, Scene, SphereGeometry, Vector3,
} from 'three';
import { CSS2DObject } from 'three/addons/renderers/CSS2DRenderer.js';
import type { ScaleModel } from './scale';

// Rendering for the navigation mission. Three trajectory layers tell the TCM
// story: the original plan (cyan reference), the current predicted arc (amber,
// rewritten by each correction burn), and the actually-flown path (bright
// white, grown as time advances). Everything is stored in heliocentric
// ecliptic AU and re-projected through the active scale model, exactly like
// the planet orbits (scene/scale.ts + eclToScene).

/** Pseudo body id used by the camera-follow machinery for the spacecraft. */
export const CRAFT_ID = '@craft';

// Ecliptic (x = equinox, z = north) -> Three.js Y-up (same as world.ts).
function eclToScene(v: Vector3, out: Vector3): Vector3 {
  return out.set(v.x, v.z, -v.y);
}

const PLAN_COLOR = 0x6fe3ff;
const PREDICTED_COLOR = 0xffb454;
const FLOWN_COLOR = 0xffffff;
const FLOWN_CAPACITY = 4096;

export class NavViz {
  readonly craft = new Group();
  /** Craft position in scene units (for camera follow). */
  readonly craftScene = new Vector3();

  private planLine: Line;
  private predictedLine: Line;
  private flownLine: Line;
  private markerStart: Mesh;
  private markerEnd: Mesh;
  private label: CSS2DObject;

  private planAU: Vector3[] = [];
  private predictedAU: Vector3[] | null = null;
  private flownRef: Vector3[] = [];
  private flownLength = 0;

  private tmpA = new Vector3();
  private tmpB = new Vector3();
  private tmpC = new Vector3();

  constructor(scene: Scene) {
    this.craft.add(
      new Mesh(
        new CylinderGeometry(0.085, 0.085, 0.4, 12).rotateX(Math.PI / 2),
        new MeshStandardMaterial({ color: 0xd7dde6, roughness: 0.5, metalness: 0.35 }),
      ),
    );
    const nose = new Mesh(
      new ConeGeometry(0.1, 0.2, 12).rotateX(Math.PI / 2),
      new MeshBasicMaterial({ color: PLAN_COLOR }),
    );
    nose.position.z = 0.3;
    const nozzle = new Mesh(
      new ConeGeometry(0.07, 0.16, 10).rotateX(-Math.PI / 2),
      new MeshStandardMaterial({ color: 0x7b8291, roughness: 0.7 }),
    );
    nozzle.position.z = -0.28;
    const panelGeo = new BoxGeometry(0.62, 0.012, 0.17);
    const panelMat = new MeshStandardMaterial({
      color: 0x2c4f7c, roughness: 0.4, metalness: 0.2, emissive: 0x0a1a2e,
    });
    const panelL = new Mesh(panelGeo, panelMat);
    panelL.position.x = -0.4;
    const panelR = new Mesh(panelGeo, panelMat);
    panelR.position.x = 0.4;
    this.craft.add(nose, nozzle, panelL, panelR);
    this.craft.visible = false;
    this.craft.frustumCulled = false;
    scene.add(this.craft);

    this.label = this.makeLabel('飞船');
    this.craft.add(this.label);

    const makeLine = (color: number, opacity: number, capacity: number) => {
      const geo = new BufferGeometry();
      geo.setAttribute('position', new Float32BufferAttribute(new Float32Array(capacity * 3), 3));
      geo.setDrawRange(0, 0);
      const line = new Line(geo, new LineBasicMaterial({ color, transparent: true, opacity }));
      line.visible = false;
      line.frustumCulled = false;
      scene.add(line);
      return line;
    };
    this.planLine = makeLine(PLAN_COLOR, 0.32, 513);
    this.predictedLine = makeLine(PREDICTED_COLOR, 0.85, 513);
    this.flownLine = makeLine(FLOWN_COLOR, 0.9, FLOWN_CAPACITY);

    this.markerStart = this.makeMarker(0x9ad9ff);
    this.markerEnd = this.makeMarker(0xffd27a);
    for (const m of [this.markerStart, this.markerEnd]) {
      m.visible = false;
      m.frustumCulled = false;
      scene.add(m);
    }
  }

  private makeLabel(text: string): CSS2DObject {
    const el = document.createElement('div');
    el.className = 'craft-label';
    el.textContent = text;
    const obj = new CSS2DObject(el);
    obj.position.set(0, 0.9, 0);
    return obj;
  }

  private makeMarker(color: number): Mesh {
    return new Mesh(
      new SphereGeometry(0.3, 14, 10),
      new MeshBasicMaterial({ color, transparent: true, opacity: 0.85 }),
    );
  }

  /** Resize a line's buffer when the point count changes. */
  private ensureCapacity(line: Line, n: number): void {
    const attr = line.geometry.getAttribute('position') as Float32BufferAttribute;
    if (attr.count >= n) return;
    line.geometry.dispose();
    const geo = new BufferGeometry();
    geo.setAttribute('position', new Float32BufferAttribute(new Float32Array(n * 3), 3));
    line.geometry = geo;
  }

  /** Original planned arc (cyan reference) + start/end markers. */
  setPlanArc(arcAU: Vector3[], startAU: Vector3, endAU: Vector3, scale: ScaleModel, f: number): void {
    this.planAU = arcAU;
    this.ensureCapacity(this.planLine, arcAU.length);
    this.planLine.geometry.setDrawRange(0, arcAU.length);
    this.project(scale, f);
    scale.position(startAU, this.tmpA);
    eclToScene(this.tmpA, this.tmpB);
    this.markerStart.position.copy(this.tmpB).setY(this.tmpB.y * f);
    scale.position(endAU, this.tmpA);
    eclToScene(this.tmpA, this.tmpB);
    this.markerEnd.position.copy(this.tmpB).setY(this.tmpB.y * f);
    this.applyVisibility();
  }

  /** Current predicted arc (amber); pass null to hide (e.g. before launch). */
  setPredicted(arcAU: Vector3[] | null, scale: ScaleModel, f: number): void {
    this.predictedAU = arcAU;
    if (arcAU) {
      this.ensureCapacity(this.predictedLine, arcAU.length);
      this.predictedLine.geometry.setDrawRange(0, arcAU.length);
      this.project(scale, f);
    }
    this.applyVisibility();
  }

  /** Actually-flown path (white). Pass the live mission array; may grow. */
  setFlown(ptsAU: Vector3[], scale: ScaleModel, f: number): void {
    this.flownRef = ptsAU;
    this.ensureCapacity(this.flownLine, Math.max(FLOWN_CAPACITY, ptsAU.length));
    this.flownLength = ptsAU.length;
    this.flownLine.geometry.setDrawRange(0, this.flownLength);
    this.project(scale, f);
    this.applyVisibility();
  }

  /** Re-project every stored trajectory through scale + flatten factor. */
  project(scale: ScaleModel, f: number): void {
    this.projectPoints(this.planAU, this.planLine, scale, f, this.planAU.length);
    if (this.predictedAU) {
      this.projectPoints(this.predictedAU, this.predictedLine, scale, f, this.predictedAU.length);
    }
    this.projectPoints(this.flownRef, this.flownLine, scale, f, this.flownLength);
  }

  private projectPoints(
    src: Vector3[],
    line: Line,
    scale: ScaleModel,
    f: number,
    count: number,
  ): void {
    if (count === 0) return;
    const attr = line.geometry.getAttribute('position') as Float32BufferAttribute;
    const arr = attr.array as Float32Array;
    const n = Math.min(count, src.length, attr.count);
    for (let k = 0; k < n; k++) {
      scale.position(src[k], this.tmpA);
      eclToScene(this.tmpA, this.tmpB);
      this.tmpB.y *= f;
      arr[k * 3] = this.tmpB.x;
      arr[k * 3 + 1] = this.tmpB.y;
      arr[k * 3 + 2] = this.tmpB.z;
    }
    attr.needsUpdate = true;
    line.geometry.setDrawRange(0, n);
  }

  private applyVisibility(): void {
    // The craft (and its label) shows whenever a mission plan exists — the
    // lines/markers each depend on their own data.
    this.craft.visible = this.planAU.length > 0;
    this.planLine.visible = this.planAU.length > 0;
    this.predictedLine.visible = !!this.predictedAU && this.predictedAU.length > 0;
    this.flownLine.visible = this.flownLength > 0;
    this.markerStart.visible = this.planAU.length > 0;
    this.markerEnd.visible = this.planAU.length > 0;
  }

  /**
   * Place the craft for the current epoch: position AU (heliocentric ecliptic)
   * and optional velocity (AU/day) for heading. Call `setCraftHover` after to
   * lift it clear of a body.
   */
  updateCraft(posAU: Vector3, velAU: Vector3 | null, scale: ScaleModel, f: number): void {
    scale.position(posAU, this.tmpA);
    eclToScene(this.tmpA, this.tmpB);
    this.tmpB.y *= f;
    this.craft.position.copy(this.tmpB);
    if (velAU) {
      this.tmpC.copy(posAU).addScaledVector(velAU, 0.5);
      scale.position(this.tmpC, this.tmpA);
      eclToScene(this.tmpA, this.tmpC);
      this.craft.lookAt(this.tmpC);
    }
    this.craftScene.copy(this.craft.position);
  }

  /** Lift the craft clear of a body's surface (scene units). */
  setCraftHover(height: number): void {
    this.craft.position.y += height;
    this.craftScene.copy(this.craft.position);
  }

  setLabel(text: string): void {
    (this.label.element as HTMLElement).textContent = text;
  }

  setVisible(on: boolean): void {
    this.craft.visible = on;
    if (on) this.applyVisibility();
    else {
      this.planLine.visible = false;
      this.predictedLine.visible = false;
      this.flownLine.visible = false;
      this.markerStart.visible = false;
      this.markerEnd.visible = false;
    }
  }

  clear(): void {
    this.planAU = [];
    this.predictedAU = null;
    this.flownRef = [];
    this.flownLength = 0;
    this.planLine.geometry.setDrawRange(0, 0);
    this.predictedLine.geometry.setDrawRange(0, 0);
    this.flownLine.geometry.setDrawRange(0, 0);
    this.setVisible(false);
  }
}
