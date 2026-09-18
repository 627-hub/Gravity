// @vitest-environment happy-dom
import { Scene, Vector3 } from 'three';
import { describe, expect, it } from 'vitest';
import { NavViz } from './nav-viz';
import { visualScale } from './scale';

// Regression: the spacecraft model (and its label / the planned arc) must
// become visible as soon as a mission plan is installed, and hide on clear.

describe('NavViz visibility', () => {
  it('shows the craft and arcs with a plan, hides on clear', () => {
    const viz = new NavViz(new Scene());
    expect(viz.craft.visible).toBe(false);

    const arc = [new Vector3(1, 0, 0), new Vector3(1.2, 0.1, 0), new Vector3(1.5, 0.2, 0)];
    viz.setPlanArc(arc, arc[0], arc[2], visualScale, 1);
    expect(viz.craft.visible).toBe(true); // the bug: this stayed false
    expect(viz.craft.children.length).toBeGreaterThan(1); // label attached

    const tri = [new Vector3(1, 0, 0), new Vector3(1.1, 0.05, 0)];
    viz.setPredicted(tri, visualScale, 1);
    viz.setFlown(tri, visualScale, 1);
    expect(viz.craft.visible).toBe(true);

    viz.updateCraft(new Vector3(1.1, 0.05, 0), new Vector3(0, 0.01, 0), visualScale, 1);
    expect(viz.craftScene.length()).toBeGreaterThan(0);

    viz.clear();
    expect(viz.craft.visible).toBe(false);
  });
});
