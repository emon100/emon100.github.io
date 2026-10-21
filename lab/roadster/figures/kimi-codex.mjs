import { t } from '../i18n.mjs';
/* Codex review of frozen Kimi k3 output; schematic coordinates, not measured dimensions. */
// kimi.mjs — Kimi's figure set for the four-way comparison.
// Ten isometric storyboard moments of a miniature sports car on an instrument
// test bench: a curved track fixture (scenes 1–5) and a flat lane with parked
// reference vehicles and a Semi-shaped truck (scenes 6–10).
//
// Everything is ILLUSTRATIVE: hypothetical layout, unverified mechanism.
// Readouts end with 示意，非实测.
//
// The car is one rigid body: every car vertex and normal is transformed by a
// true 3D world-space rotation + translation per progress value. No SVG
// screen-space rotation is ever used. Rendering preserves the Anatomy
// four-tone style with a small explicit world-vertex polyhedron renderer
// (the kit's floor-standing prisms cannot express an inverted rigid body),
// with measured painter's order via Newell plane-separation tests.

import * as k from '../kit/iso-kit.mjs';

/* ------------------------------------------------------------------ */
/* 3D helpers                                                          */
/* ------------------------------------------------------------------ */

const DEG = Math.PI / 180;
const dot3 = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const sub3 = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const centroidOf = (pts) => {
  const c = [0, 0, 0];
  for (const p of pts) { c[0] += p[0]; c[1] += p[1]; c[2] += p[2]; }
  return [c[0] / pts.length, c[1] / pts.length, c[2] / pts.length];
};

// Row-major 3x3 rotations.
const rotX = (g) => { const c = Math.cos(g * DEG), s = Math.sin(g * DEG); return [1, 0, 0, 0, c, -s, 0, s, c]; };
const rotY = (b) => { const c = Math.cos(b * DEG), s = Math.sin(b * DEG); return [c, 0, s, 0, 1, 0, -s, 0, c]; };
const rotZ = (a) => { const c = Math.cos(a * DEG), s = Math.sin(a * DEG); return [c, -s, 0, s, c, 0, 0, 0, 1]; };
const IDR = [1, 0, 0, 0, 1, 0, 0, 0, 1];
const applyM = (M, v) => [
  M.r[0] * v[0] + M.r[1] * v[1] + M.r[2] * v[2] + M.t[0],
  M.r[3] * v[0] + M.r[4] * v[1] + M.r[5] * v[2] + M.t[1],
  M.r[6] * v[0] + M.r[7] * v[1] + M.r[8] * v[2] + M.t[2],
];
const applyR = (r, v) => [
  r[0] * v[0] + r[1] * v[1] + r[2] * v[2],
  r[3] * v[0] + r[4] * v[1] + r[5] * v[2],
  r[6] * v[0] + r[7] * v[1] + r[8] * v[2],
];

/* ------------------------------------------------------------------ */
/* Camera: front of the ramp, original lane view, shared object scale.*/
/* ------------------------------------------------------------------ */

const VW = 640, VH = 430, PAD = 30, AZ = 45, RAMP_AZ = 140, EL = 30;
const viewerOf = P => {const c=k.cameraOf(P);return [c.cosA*c.cosE,c.sinA*c.cosE,c.sinE]};

const REGIONS = {
  A: [...k.boxCorners({ x: -165, y: -85, w: 210, d: 170 }, -17, 0), [-55, 0, 182]],
  B: [...k.boxCorners({ x: -125, y: -100, w: 440, d: 200 }, -17, 0), [96, 0, 110]],
};
const SCALE = Math.min(
  k.fitProjection(REGIONS.A, VW, VH, { pad: PAD, azimuth: RAMP_AZ, elevation: EL }).scale,
  k.fitProjection(REGIONS.B, VW, VH, { pad: PAD, azimuth: AZ, elevation: EL }).scale,
);
const projFor = (scene) => {
  const pts = REGIONS[scene <= 5 ? 'A' : 'B'], azimuth=scene<=5?RAMP_AZ:AZ;
  const unit = { origin: [0, 0], scale: 1, azimuth, elevation: EL };
  const flat = pts.map((p) => k.iso(p, unit));
  const xs = flat.map((p) => p[0]), ys = flat.map((p) => p[1]);
  const l = Math.min(...xs), r = Math.max(...xs), u = Math.min(...ys), d = Math.max(...ys);
  return { origin: [VW / 2 - ((l + r) / 2) * SCALE, VH / 2 - ((u + d) / 2) * SCALE], scale: SCALE, azimuth, elevation: EL };
};

/* ------------------------------------------------------------------ */
/* Polyhedron parts: convex prisms / wedges / n-gon cylinders, built   */
/* in local coordinates and given a rigid world transform.             */
/* ------------------------------------------------------------------ */

// Prism from a 2D ring extruded along 'x' | 'y' | 'z' between a and b.
function extrudeAlong(ring, axis, a, b) {
  const n = ring.length;
  const pt = (uv, s) => (axis === 'z' ? [uv[0], uv[1], s] : axis === 'y' ? [uv[0], s, uv[1]] : [s, uv[0], uv[1]]);
  const v = [...ring.map((uv) => pt(uv, a)), ...ring.map((uv) => pt(uv, b))];
  const f = [[...Array(n).keys()], [...Array(n).keys()].map((i) => n + i)];
  for (let i = 0; i < n; i++) f.push([i, (i + 1) % n, n + ((i + 1) % n), n + i]);
  return { v, f };
}

// Box, optionally tapered in y and sloped in z along x (all faces planar).
function wedgeGeo({ x0, x1, y0a, y1a, y0b, y1b, z0a, z0b, z1a, z1b }) {
  const v = [
    [x0, y0a, z0a], [x0, y1a, z0a], [x1, y0b, z0b], [x1, y1b, z0b],
    [x0, y0a, z1a], [x0, y1a, z1a], [x1, y0b, z1b], [x1, y1b, z1b],
  ];
  const f = [[0, 1, 3, 2], [4, 6, 7, 5], [0, 2, 6, 4], [1, 5, 7, 3], [0, 4, 5, 1], [2, 3, 7, 6]];
  return { v, f };
}
const box = (x0, x1, y0, y1, z0, z1) => wedgeGeo({ x0, x1, y0a: y0, y1a: y1, y0b: y0, y1b: y1, z0a: z0, z0b: z0, z1a: z1, z1b: z1 });

function cylGeo(axis, cu, cv, from, to, r, n = 12) {
  const ring = [];
  for (let i = 0; i < n; i++) {
    const t = (i / n) * 2 * Math.PI;
    ring.push([cu + r * Math.cos(t), cv + r * Math.sin(t)]);
  }
  return extrudeAlong(ring, axis, from, to);
}

function newell(pts) {
  let nx = 0, ny = 0, nz = 0;
  for (let i = 0; i < pts.length; i++) {
    const a = pts[i], b = pts[(i + 1) % pts.length];
    nx += (a[1] - b[1]) * (a[2] + b[2]);
    ny += (a[2] - b[2]) * (a[0] + b[0]);
    nz += (a[0] - b[0]) * (a[1] + b[1]);
  }
  const l = Math.hypot(nx, ny, nz);
  return l < 1e-12 ? null : [nx / l, ny / l, nz / l];
}

function hull2(pts) {
  const p = [...pts].sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  if (p.length < 3) return p;
  const cross = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const lo = [], up = [];
  for (const q of p) { while (lo.length >= 2 && cross(lo[lo.length - 2], lo[lo.length - 1], q) <= 0) lo.pop(); lo.push(q); }
  for (let i = p.length - 1; i >= 0; i--) { const q = p[i]; while (up.length >= 2 && cross(up[up.length - 2], up[up.length - 1], q) <= 0) up.pop(); up.push(q); }
  lo.pop(); up.pop();
  return lo.concat(up);
}

/* ------------------------------------------------------------------ */
/* Part renderer (Anatomy four-tone: paper hull, tone by facing,       */
/* silhouette outline, creases on hard top joins).                     */
/* ------------------------------------------------------------------ */

const TOPCOS = 0.72;       // |nz| above this reads as a top face
const CREASE = 0.94;       // cos(dihedral) below this (with a top face) creases
const HARD = 0.5;          // any both-visible join sharper than 60° strokes

function detailSvg(det, M, P) {
  if (det.n) {
    const nw = applyR(M.r, det.n);
    if (dot3(nw, viewerOf(P)) <= 0.05) return '';
  }
  if (det.type === 'line') return k.lineSvg(k.pathOf(det.pts.map((p) => k.iso(applyM(M, p), P))), { tone: det.tone ?? 'faint' });
  if (det.type === 'dot') return k.dotsSvg([k.iso(applyM(M, det.at), P)], { size: det.size ?? 0.45, tone: det.tone ?? 'mid' });
  return '';
}

function buildItem(def, M, P) {
  const V3=viewerOf(P), c=k.cameraOf(P), L2=[-c.sinA,c.cosA];
  const wv = def.geo.v.map((v) => applyM(M, v));
  const pc = centroidOf(wv);
  const sp = wv.map((p) => k.iso(p, P));
  const faces = [];
  def.geo.f.forEach((idx, fi) => {
    if (def.skipFaces && def.skipFaces.includes(fi)) return;
    const pts = idx.map((i) => wv[i]);
    let n = newell(pts);
    if (!n) return;
    if (dot3(n, sub3(centroidOf(pts), pc)) < 0) n = [-n[0], -n[1], -n[2]];
    faces.push({ idx, pts, n, c: centroidOf(pts), vis: dot3(n, V3) > 1e-9 });
  });
  // Edges: silhouette (visible XOR) and creases.
  const edges = new Map();
  faces.forEach((f) => {
    const m = f.idx.length;
    for (let i = 0; i < m; i++) {
      const a = f.idx[i], b = f.idx[(i + 1) % m];
      const key = a < b ? `${a}_${b}` : `${b}_${a}`;
      let e = edges.get(key);
      if (!e) { e = { a, b, fs: [] }; edges.set(key, e); }
      e.fs.push(f);
    }
  });
  const outline = [], crease = [];
  for (const e of edges.values()) {
    if (e.fs.length !== 2) continue;
    const [f1, f2] = e.fs;
    const pa = k.iso(wv[e.a], P), pb = k.iso(wv[e.b], P);
    if (f1.vis !== f2.vis) outline.push(k.pathOf([pa, pb]));
    else if (f1.vis) {
      const cosD = dot3(f1.n, f2.n);
      const topish = Math.abs(f1.n[2]) > TOPCOS || Math.abs(f2.n[2]) > TOPCOS;
      if ((topish && cosD < CREASE) || cosD < HARD) crease.push(k.pathOf([pa, pb]));
    }
  }
  // Depth envelope for sorting.
  let dmin = Infinity, dmax = -Infinity;
  let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
  for (let i = 0; i < wv.length; i++) {
    const d = k.depthOf(wv[i], P);
    if (d < dmin) dmin = d;
    if (d > dmax) dmax = d;
    const [sx, sy] = sp[i];
    if (sx < x0) x0 = sx; if (sx > x1) x1 = sx;
    if (sy < y0) y0 = sy; if (sy > y1) y1 = sy;
  }
  const planes = faces.map((f) => ({ n: f.n, d: dot3(f.n, f.pts[0]) }));
  // Paint: paper hull, then visible faces back to front, creases, outline.
  const chunks = [`<path class="iso-fill" d="${k.pathOf(hull2(sp), true)}"/>`];
  const vis = faces.filter((f) => f.vis).sort((a, b) => k.depthOf(a.c, P) - k.depthOf(b.c, P));
  for (const f of vis) {
    const d = k.pathOf(f.pts.map((p) => k.iso(p, P)), true);
    if (Math.abs(f.n[2]) > TOPCOS) chunks.push(`<path class="iso-top" d="${d}"/>`);
    else {
      const lit = (f.n[0] * L2[0] + f.n[1] * L2[1] + 1) / 2;
      const bucket = Math.min(3, Math.max(0, Math.floor(lit * 4)));
      chunks.push(`<path class="iso-shade" data-shade="${bucket}" d="${d}"/>`);
    }
  }
  if (crease.length) chunks.push(`<path class="iso-line iso-crease" data-tone="faint" d="${crease.join('')}"/>`);
  chunks.push(`<path class="iso-line iso-edge" data-tone="${def.tone ?? 'hi'}" d="${outline.join('')}"/>`);
  for (const det of def.details ?? []) chunks.push(detailSvg(det, M, P));
  const svg = `<g class="iso-solid"${def.material ? ` data-material="${def.material}"` : ''}${def.lit ? ' data-lit' : ''}>${chunks.join('')}</g>`;
  return { svg, wv, planes, viewer:V3, dmin, dmax, cd: k.depthOf(pc, P), bbox: [x0, y0, x1, y1] };
}

// Measured painter's order: centroid depth, then Newell plane tests.
const SEP_EPS = 0.02;
function orderItems(items) {
  items.sort((a, b) => a.cd - b.cd);
  const behind = (A, B) => {
    const V3=A.viewer;
    if (A.dmax <= B.dmin + SEP_EPS) return true;
    const [ax0, ay0, ax1, ay1] = A.bbox, [bx0, by0, bx1, by1] = B.bbox;
    if (ax0 > bx1 || bx0 > ax1 || ay0 > by1 || by0 > ay1) return true;
    // A must be outside a BACK face of B, not merely behind a front
    // face that can span beneath A. The latter incorrectly hid the canopy.
    for (const pl of B.planes) {
      if (dot3(pl.n, V3) >= -1e-6) continue;
      let ok = true;
      for (const v of A.wv) if (dot3(pl.n, v) < pl.d - SEP_EPS) { ok = false; break; }
      if (ok) return true;
    }
    for (const pl of A.planes) {
      if (dot3(pl.n, V3) <= 1e-6) continue;
      let ok = true;
      for (const v of B.wv) if (dot3(pl.n, v) < pl.d - SEP_EPS) { ok = false; break; }
      if (ok) return true;
    }
    return false;
  };
  for (let pass = 0; pass < 8; pass++) {
    let moved = false;
    for (let i = 0; i < items.length; i++) {
      for (let j = i + 1; j < items.length; j++) {
        if (!behind(items[i], items[j]) && behind(items[j], items[i])) {
          const t = items[i]; items[i] = items[j]; items[j] = t;
          moved = true;
        }
      }
    }
    if (!moved) break;
  }
  return items;
}

/* ------------------------------------------------------------------ */
/* Rocket prototype exterior: 56-unit envelope, 34-unit wheelbase.     */
/* The body is split around real open wheel wells; the flight footage */
/* guides the disc appearance; blur leaves the actual wheel construction unknown.     */
/* Parts: narrow pan; four arch crowns in eight convex strips each;    */
/* five flank panels/side; bonnet/nose/lip; glass canopy + six frame   */
/* rails; four tyre/shoulder/rim/cover/hub sets; four cold-gas outlets;  */
/* four separate fan ducts/hubs; two wing supports and a full-span wing;*/
/* diffuser blades, side skirts and four inboard downward outlets.    */
/* ------------------------------------------------------------------ */

const CAR_AXLE_HALF = 17, CAR_TIRE_R = 6;
function frustumGeo(axis, cu, cv, from, to, r0, r1, n = 24) {
  const at = (u,v,s) => axis === 'x' ? [s,u,v] : axis === 'y' ? [u,s,v] : [u,v,s];
  const v = [r0,r1].flatMap((radius,j) => Array.from({length:n},(_,i) => {
    const a=i*2*Math.PI/n;return at(cu+radius*Math.cos(a),cv+radius*Math.sin(a),j?to:from);
  }));
  return {v,f:[[...Array(n).keys()],[...Array(n).keys()].map(i=>i+n),...Array.from({length:n},(_,i)=>[i,(i+1)%n,(i+1)%n+n,i+n])]};
}

function carParts() {
  const parts=[];
  const add=(name,geo,material='paint',details=[])=>parts.push({name,geo,material,tone:material==='rubber'||material==='carbon'?'lo':'hi',details});
  const B=(name,x0,x1,y0,y1,z0,z1,material='paint',details=[])=>add(name,box(x0,x1,y0,y1,z0,z1),material,details);
  const side=(name,ring,y0,y1,material='paint',details=[])=>add(name,extrudeAlong(ring,'y',y0,y1),material,details);
  const flankTop=x=>x<-10.5?14.3:x>23.5?13.3-(x-23.5)*1.2:x>10.5?13.3:13.7-(x+10.5)*.019;
  B('pan',-25,25,-8.7,8.7,4.3,5.0,'carbon');
  B('inner.body',-25,25,-8.7,8.7,5,10.7);
  add('rear.deck',wedgeGeo({x0:-28,x1:-18.7,y0a:-11.5,y1a:11.5,y0b:-9.7,y1b:9.7,z0a:12.3,z0b:10.7,z1a:14.3,z1b:14.3}));
  add('hood',wedgeGeo({x0:9.5,x1:27.7,y0a:-9.7,y1a:9.7,y0b:-8.7,y1b:8.7,z0a:10.7,z0b:5.3,z1a:12.4,z1b:7.9}));
  for(const sy of [-1,1]){
    const y0=sy<0?-12.4:8.8,y1=sy<0?-8.8:12.4;
    for(const [a,b] of [[-28,-23.5],[-10.5,10.5],[23.5,28]]){
      const lowa=a< -23?5:a>23?4.7:5.1,lowb=b>23?5.2:b< -23?5:4.8;
      side(`flank.${sy}.${a}`,[[a,lowa],[b,lowb],[b,flankTop(b)],[a,flankTop(a)]],y0,y1);
    }
    for(const axle of [-CAR_AXLE_HALF,CAR_AXLE_HALF]){
      const radius=6.5,steps=8;
      for(let j=0;j<steps;j++){
        const a=Math.PI-j*Math.PI/steps,b=Math.PI-(j+1)*Math.PI/steps;
        const x0=axle+radius*Math.cos(a),x1=axle+radius*Math.cos(b);
        const z0=6+radius*Math.sin(a),z1=6+radius*Math.sin(b);
        side(`arch.${axle}.${sy}.${j}`,[[x0,z0],[x1,z1],[x1,flankTop(x1)],[x0,flankTop(x0)]],y0,y1);
        parts.at(-1).skipFaces=[3,5]; // Adjacent convex strips share these internal cuts.
      }
    }
    B(`sill${sy}`,-10.45,10.45,sy<0?-12.45:11.4,sy<0?-11.4:12.45,3.7,5.05,'carbon');
    side(`door.seam${sy}`,[[-6.2,5.1],[-6.05,5.1],[-5.4,13.6],[-5.55,13.6]],sy<0?-12.43:12.4,sy<0?-12.4:12.43,'carbon');
    B(`camera.stalk${sy}`,8.2,10.0,sy<0?-13.2:11.8,sy<0?-11.8:13.2,12.5,12.9,'carbon');
    B(`camera.head${sy}`,8.6,10.2,sy<0?-14:13.2,sy<0?-13.2:14,12.6,13.2,'paint');
    // The hidden lamp lids resolve into the bonnet shoulders.
    side(`lamp.lid${sy}`,[[22.5,11.8],[25.8,9.2],[24.4,10.6]],sy<0?-11.7:9.6,sy<0?-9.6:11.7,'metal');
  }
  B('nose.upper',27.65,28,-11.3,11.3,6.0,7.9,'paint');
  B('nose.intake',27.9,28.02,-8.6,8.6,4.8,5.95,'carbon');
  B('splitter',24.0,28.05,-11.8,11.8,4.35,4.7,'carbon');
  B('nose.light.strip',28.02,28.08,-8.75,8.75,7.65,7.82,'metal');

  // One continuous dark canopy; no unverified passenger-seat interior.
  const canopy={v:[[9.5,-9.7,12.4],[9.5,9.7,12.4],[-18.7,-9.7,14.3],[-18.7,9.7,14.3],[-1.0,-8.25,18.15],[-1.0,8.25,18.15],[-10.5,-8.0,18.0],[-10.5,8.0,18.0]],f:[[0,2,3,1],[0,1,5,4],[2,6,7,3],[0,4,6,2],[1,3,7,5],[4,5,7,6]]};
  add('continuous.canopy',canopy,'glass');
  for(const sy of [-1,1]){
    const rail=(name,x0,x1,w0,w1,z0,z1)=>add(name,wedgeGeo({x0,x1,y0a:sy<0?-w0-.22:w0,y1a:sy<0?-w0:w0+.22,y0b:sy<0?-w1-.22:w1,y1b:sy<0?-w1:w1+.22,z0a:z0,z0b:z1,z1a:z0+.24,z1b:z1+.24}));
    rail(`canopy.A${sy}`,-1,9.5,8.25,9.7,18.15,12.4);
    rail(`canopy.roof${sy}`,-10.5,-1,8,8.25,18.0,18.15);
    rail(`canopy.C${sy}`,-18.7,-10.5,9.7,8,14.3,18.0);
  }
  for(const axle of [-CAR_AXLE_HALF,CAR_AXLE_HALF])for(const sy of [-1,1]){
    add(`tire${axle}_${sy}`,cylGeo('y',axle,6,sy>0?9.4:-12.08,sy>0?12.08:-9.4,6,24),'rubber');
    add(`shoulder${axle}_${sy}`,frustumGeo('y',axle,6,sy>0?12.08:-12.35,sy>0?12.35:-12.08,sy>0?6:5.75,sy>0?5.75:6,24),'rubber');
    add(`rim${axle}_${sy}`,cylGeo('y',axle,6,sy>0?12.35:-12.43,sy>0?12.43:-12.35,5.12,24),'metal');
    add(`cover${axle}_${sy}`,frustumGeo('y',axle,6,sy>0?12.43:-12.58,sy>0?12.58:-12.43,sy>0?4.75:4.65,sy>0?4.65:4.75,24),'metal');
    add(`hub${axle}_${sy}`,cylGeo('y',axle,6,sy>0?12.58:-12.65,sy>0?12.65:-12.58,.68,16),'carbon');
  }

  // Four small cold-gas exits above four distinct, larger fan ducts.
  B('rear.fascia',-27.9,-26.6,-11.8,11.8,8.85,13.8,'carbon');
  B('rear.lamp',-28.02,-27.9,-11.5,11.5,13.45,13.8,'metal');
  B('rear.plate',-28.06,-27.92,-2.6,2.6,11.3,13.3,'paint');
  for(const y of [-8.1,-2.7,2.7,8.1]){
    add(`rear.coldgas.case${y}`,cylGeo('x',y,10.2,-28.02,-25.8,1.25,16),'carbon');
    add(`rear.coldgas.lip${y}`,cylGeo('x',y,10.2,-28.15,-28.02,1.33,20),'metal');
    add(`rear.coldgas.bore${y}`,cylGeo('x',y,10.2,-28.16,-28.15,1.04,20),'void');
    B(`fan.cell${y}`,-27.7,-25.5,y-2.5,y+2.5,4.8,8.9,'carbon');
    add(`rear.fan.rim${y}`,cylGeo('x',y,6.8,-27.95,-27.7,2.12,20),'carbon');
    add(`rear.fan.throat${y}`,cylGeo('x',y,6.8,-27.98,-27.95,1.86,20),'void');
    add(`rear.fan.hub${y}`,frustumGeo('x',y,6.8,-28.08,-27.98,.66,.96,16),'carbon');
  }
  B('rear.diffuser',-27.9,-22,-10.8,10.8,4.9,5.2,'carbon');
  for(const y of [-8.2,-4.1,0,4.1,8.2])B(`diffuser.fin${y}`,-27.8,-22,y-.12,y+.12,4.6,4.9,'carbon');
  for(const y of [-7.8,7.8])side(`wing.support${y}`,[[-26.1,14.3],[-25.0,14.3],[-23.8,18.1],[-24.7,18.1]],y-.4,y+.4,'carbon');
  add('rear.wing',wedgeGeo({x0:-28,x1:-22.7,y0a:-12.8,y1a:12.8,y0b:-12.8,y1b:12.8,z0a:18.1,z0b:18.1,z1a:18.8,z1b:18.35}),'carbon');
  for(const sy of [-1,1])B(`wing.endplate${sy}`,-28,-22.7,sy<0?-13.0:12.8,sy<0?-12.8:13.0,17.7,19.0,'carbon');
  for(const [x,y] of [[14.5,8.3],[14.5,-8.3],[-19.3,8.3],[-19.3,-8.3]]){
    B(`pod${x}_${y}`,x-1.3,x+1.3,y-1.2,y+1.2,2.5,4.3,'carbon');
    add(`nozzle${x}_${y}`,cylGeo('z',x,y,1.4,2.5,1.12,16),'metal');
    add(`tip${x}_${y}`,frustumGeo('z',x,y,.7,1.4,.78,1.12,16),'metal');
  }
  return parts;
}
const CAR_DEFS=carParts();
const NOZZLES=[[14.5,8.3],[14.5,-8.3],[-19.3,8.3],[-19.3,-8.3]];

/* ------------------------------------------------------------------ */
/* Environment parts                                                   */
/* ------------------------------------------------------------------ */

const LOOP = { cx: -55, cz: 66, R: 60, band: 6, endAngle: 25, extension: 95 };
const DECK_Z = 6, TIRE_R = CAR_TIRE_R, AXLE_HALF = CAR_AXLE_HALF;
const ARC_LENGTH = LOOP.R * (LOOP.endAngle + 90) * DEG;
const RO = TIRE_R + Math.sqrt((LOOP.R - TIRE_R) ** 2 - AXLE_HALF ** 2);
const TRACK_END = ARC_LENGTH + LOOP.extension;
const HOLD_FRONT = ARC_LENGTH + 78;
const PAD_Z = 0.8;

// Surface station -> tire centre; offset follows the actual surface normal.
function wheelPath(s) {
  if (s < 0) return [LOOP.cx + s, 0, DECK_Z + TIRE_R];
  const a = (-90 + Math.min(s, ARC_LENGTH) / LOOP.R / DEG) * DEG;
  const r = LOOP.R - TIRE_R;
  const p = [LOOP.cx + r * Math.cos(a), 0, LOOP.cz + r * Math.sin(a)];
  if (s > ARC_LENGTH) {
    p[0] -= Math.sin(a) * (s - ARC_LENGTH);
    p[2] += Math.cos(a) * (s - ARC_LENGTH);
  }
  return p;
}
function supportedPose(frontS) {
  const F = wheelPath(frontS);
  let lo = frontS - 130, hi = frontS;
  for (let i = 0; i < 45; i++) {
    const m = (lo + hi) / 2, B = wheelPath(m);
    if (Math.hypot(F[0] - B[0], F[2] - B[2]) > AXLE_HALF * 2) lo = m;
    else hi = m;
  }
  const rearS = (lo + hi) / 2, B = wheelPath(rearS);
  const pitch = Math.atan2(F[2] - B[2], F[0] - B[0]) / DEG;
  const r = rotY(-pitch), axleMid = F.map((v, i) => (v + B[i]) / 2);
  const z = applyR(r, [0, 0, TIRE_R]);
  return { M: { r, t: axleMid.map((v, i) => v - z[i]) }, beta: -pitch, frontS, rearS, gap: 0 };
}
function poseAtCentre(r, C, extras = {}) {
  const offset = applyR(r, [0, 0, 10]);
  return { M: { r, t: C.map((v, i) => v - offset[i]) }, ...extras };
}
function mulR(a, b) {
  return Array.from({length: 9}, (_, i) => [0, 1, 2].reduce((v, j) => v + a[3 * Math.floor(i / 3) + j] * b[3 * j + i % 3], 0));
}

function fixtureParts() {
  const parts = [];
  for (const x of [-140, -105, -70]) parts.push({ name: 'sleeper', tone: 'lo', geo: box(x - 3, x + 3, -30, 30, 0, 3) });
  const deckDots = [-145, -125, -105, -85, -65].map((x) => ({ type: 'dot', n: [0, 1, 0], at: [x, 28.15, 4.5], size: 0.45, tone: 'lo' }));
  parts.push({ name: 'deck', tone: 'mid', geo: box(-150, -55, -28, 28, 3, 6), details: [
    { type: 'line', n: [0, 1, 0], pts: [[-148, 28.15, 4.5], [-57, 28.15, 4.5]], tone: 'faint' },
    ...deckDots,
    { type: 'line', n: [0, 0, 1], pts: [[-148, 16, 6.12], [-78, 16, 6.12], [-78, -16, 6.12], [-148, -16, 6.12], [-148, 16, 6.12]], tone: 'faint' },
    ...[[-148, 16], [-148, -16], [-78, 16], [-78, -16]].map(([x, y]) => ({ type: 'dot', n: [0, 0, 1], at: [x, y, 6.12], size: 0.5, tone: 'lo' })),
  ] });
  // Curved lower ramp followed by a straight surface 25° past vertical.
  // Station geometry is shared with the wheel-contact construction.
  const trackStations = Array.from({length: 25}, (_, i) => ARC_LENGTH * i / 24);
  trackStations.push(TRACK_END);
  const surf = (station, radius) => {
    const a = (-90 + Math.min(station, ARC_LENGTH) / LOOP.R / DEG) * DEG;
    const tail = Math.max(0, station - ARC_LENGTH);
    return [LOOP.cx + radius * Math.cos(a) - tail * Math.sin(a), LOOP.cz + radius * Math.sin(a) + tail * Math.cos(a)];
  };
  for (let i = 0; i < trackStations.length - 1; i++) {
    const a = trackStations[i], b = trackStations[i + 1];
    const ring = [surf(a, LOOP.R), surf(b, LOOP.R), surf(b, LOOP.R + LOOP.band), surf(a, LOOP.R + LOOP.band)];
    const skipFaces = [3, 5].filter(f => !((i === 0 && f === 5) || (i === trackStations.length - 2 && f === 3)));
    parts.push({ name: `track${i}`, tone: 'mid', geo: extrudeAlong(ring, 'y', -28, 28), skipFaces });
  }
  const zOut = (x) => LOOP.cz - Math.sqrt((LOOP.R + 6) ** 2 - (x - LOOP.cx) ** 2);
  parts.push({ name: 'post1', tone: 'mid', geo: wedgeGeo({ x0: -22, x1: -18, y0a: -3.5, y1a: 3.5, y0b: -3.5, y1b: 3.5, z0a: 0, z0b: 0, z1a: zOut(-22), z1b: zOut(-18) }) });
  parts.push({ name: 'post2', tone: 'mid', geo: wedgeGeo({ x0: -4, x1: 0, y0a: -3.5, y1a: 3.5, y0b: -3.5, y1b: 3.5, z0a: 0, z0b: 0, z1a: zOut(-4), z1b: zOut(0) }) });
  parts.push({ name: 'tie', tone: 'lo', geo: box(-22, 0, -3, 3, 0, 5) });
  for (const y of [-35, 35]) {
    parts.push({ name: 'backpost', tone: 'mid', geo: box(10, 15, y - 2, y + 2, 0, 145) });
    parts.push({ name: 'brace', tone: 'lo', geo: wedgeGeo({x0: -41, x1: 13, y0a: y-2, y1a: y+2, y0b:y-2, y1b:y+2, z0a:170, z0b:135, z1a:174,z1b:139}) });
  }
  return parts;
}

const PARKED = [0, 47, 145, 192];   // reference car centres on the lane
const PARKED_HALF = 18.6, PARKED_ROOF = 17, PARKED_YHALF = 32;
const SEMI = { x0: 76, x1: 116, roof: 55, yhalf: 32 };

function parkedCar(cx) {
  const m = { r: rotZ(90), t: [cx, 0, 0] };
  const parts = [
    { name: 'pc.body', tone: 'mid', geo: wedgeGeo({ x0: -16, x1: 16, y0a: -8, y1a: 8, y0b: -5.5, y1b: 5.5, z0a: 2, z0b: 2, z1a: 5.5, z1b: 5.5 }) },
    { name: 'pc.cab', tone: 'mid', geo: box(-7, 5, -6, 6, 5.5, 8.5), details: [
      { type: 'line', n: [0, 0, 1], pts: [[-5, 4.5, 8.65], [3, 4.5, 8.65], [3, -4.5, 8.65], [-5, -4.5, 8.65], [-5, 4.5, 8.65]] },
    ] },
  ];
  for (const wx of [10, -10]) for (const sy of [1, -1]) {
    parts.push({ name: 'pc.tire', tone: 'lo', geo: cylGeo('y', wx, 2.8, sy > 0 ? 6.7 : -9.3, sy > 0 ? 9.3 : -6.7, 2.8, 10), details: [
      { type: 'dot', n: [0, sy, 0], at: [wx, sy * 9.45, 2.8], size: 0.4, tone: 'mid' },
    ] });
  }
  return parts.map((p) => ({ ...p, name: p.name + cx, geo: {...p.geo, v: p.geo.v.map(v => v.map(x => x * 2))}, details: (p.details || []).map(d => ({...d, at: d.at?.map(x=>x*2), pts: d.pts?.map(v=>v.map(x=>x*2)), size: d.size ? d.size * 2 : undefined})), m }));
}

function semiParts() {
  // Cab-only Semi reference, facing +Y like the four adjacent cars.
  const parts = [
    {name:'semi.chassis',tone:'lo',geo:box(79,113,-31,27,5,12)},
    {name:'semi.cab',tone:'mid',geo:box(77,115,-15,24,12,48)},
    {name:'semi.roof',tone:'mid',geo:box(79,113,-13,17,48,55)},
    {name:'semi.glass',tone:'lo',geo:box(80,112,24,24.2,30,45)},
    {name:'semi.bumper',tone:'mid',geo:box(77,115,24,28,10,15)},
    {name:'semi.leftlamp',tone:'hi',geo:box(79,85,28,28.3,15,18)},
    {name:'semi.rightlamp',tone:'hi',geo:box(107,113,28,28.3,15,18)},
  ];
  for (const x of [77,115]) for (const y of [-22,17]) parts.push({name:'semi.wheel'+x+y,tone:'lo',geo:cylGeo('x',y,7,x-3,x+3,7,12)});
  return parts;
}

function laneParts(scene) {
  const parts = [];
  const pad = (x0, x1) => ({ name: 'pad', tone: 'lo', geo: box(x0, x1, -18, 18, 0, 0.8), details: [
    { type: 'line', n: [0, 0, 1], pts: [[x0 + 4, 14, 0.95], [x1 - 4, 14, 0.95], [x1 - 4, -14, 0.95], [x0 + 4, -14, 0.95], [x0 + 4, 14, 0.95]], tone: 'faint' },
    ...[[x0 + 4, 14], [x0 + 4, -14], [x1 - 4, 14], [x1 - 4, -14]].map(([x, y]) => ({ type: 'dot', n: [0, 0, 1], at: [x, y, 0.95], size: 0.5, tone: 'mid' })),
  ] });
  parts.push(pad(-117, -33));           // takeoff pad, present on the whole lane
  parts.push(pad(230, 310));           // landing pad
  for (const cx of PARKED) parts.push(...parkedCar(cx));
  parts.push(...semiParts());
  return parts;
}

const envParts = (scene) => (scene <= 5 ? fixtureParts() : laneParts(scene));

/* ------------------------------------------------------------------ */
/* Bench (kit floor-standing solids, always below everything else)     */
/* ------------------------------------------------------------------ */

function benchSvg(scene, P) {
  const isA = scene <= 5;
  const PLATE = isA ? { x: -165, y: -85, w: 210, d: 170, r: 8 } : { x: -125, y: -100, w: 440, d: 200, r: 8 };
  const out = [`<path class="iso-halo" d="${k.haloOf(PLATE, -17, 12, P)}"/>`];
  const feet = k.corners(PLATE, 22);
  if (!isA) feet.push([75, PLATE.y + 22], [75, PLATE.y + PLATE.d - 22]);
  const byDepth = (pts) => [...pts].sort((a, b) => k.depthOf([a[0], a[1], 0], P) - k.depthOf([b[0], b[1], 0], P));
  for (const [x, y] of byDepth(feet)) {
    out.push(k.solidSvg(k.cylinder(x, y, 7, -17, 1.6, P, 28), { tone: 'lo' }));
    out.push(k.solidSvg(k.cylinder(x, y, 5.4, -15.4, 3.4, P, 28), { tone: 'mid' }));
  }
  out.push(k.solidSvg(k.slabOf(PLATE, -12, 12, P, 8, 1.6), { tone: 'mid' }));
  out.push(k.lineSvg(k.planOutline(k.insetPlan(PLATE, 5), 0, P), { tone: 'lo' }));
  const screws = k.corners(PLATE, 10);
  out.push(k.lineSvg(screws.map(([x, y]) => k.ring(x, y, 2.2, 0, P, 16)).join(''), { tone: 'lo' }));
  out.push(k.dotsSvg(screws.map(([x, y]) => k.iso([x, y, 0], P)), { size: 0.5 }));
  const strip = { x: PLATE.x + 10, y: PLATE.y + PLATE.d - 15, w: PLATE.w - 20, d: 7, r: 1 };
  out.push(k.solidSvg(k.slabOf(strip, 0, 0.6, P, 3), { tone: 'lo', crease: 'none' }));
  const ticks = k.topTicks(strip.x + 4, strip.x + strip.w - 4, 4, 5, strip.y + 0.8, 0.6, [2, 3.6], P);
  out.push(k.lineSvg(ticks.minor, { tone: 'lo' }), k.lineSvg(ticks.major, { tone: 'mid' }));
  out.push(k.dotsSvg([k.iso([strip.x + 2, strip.y + 3.5, 0.6], P), k.iso([strip.x + strip.w - 2, strip.y + 3.5, 0.6], P)], { size: 0.45 }));
  const label = { x: PLATE.x + 12, y: PLATE.y + PLATE.d - 34, w: 34, d: 12, r: 2 };
  out.push(k.solidSvg(k.slabOf(label, 0, 0.8, P, 4), { tone: 'mid' }));
  out.push(k.lineSvg(k.planOutline(k.insetPlan(label, 1.6), 0.8, P), { tone: 'faint' }));
  for (let i = 0; i < 3; i++) {
    out.push(k.lineSvg(k.lineOnTop([label.x + 4, label.y + 3.4 + i * 2.6], [label.x + 4 + [24, 18, 21][i], label.y + 3.4 + i * 2.6], 0.85, P), { tone: 'lo', free: true }));
  }
  out.push(k.dotsSvg([k.iso([label.x + 2.4, label.y + 2.4, 0.8], P), k.iso([label.x + label.w - 2.4, label.y + 2.4, 0.8], P)], { size: 0.45 }));
  // The single tiny engraved label.
  out.push(k.faceTextSvg(k.topMatrix([label.x + 17, label.y + 8.6], 0.8, P, 'x'), t('示意', 'Schematic'), { size: 2.6, tone: 'lo', anchor: 'middle' }));
  // Static halos grounding the furniture.
  if (scene <= 5) {
    out.push(`<path class="iso-halo" d="${k.haloOf({ x: -83, y: 28, w: 18, d: 18, r: 4 }, 0, 0.1, P)}"/>`);
    out.push(`<path class="iso-halo" d="${k.haloOf({ x: -150, y: -30, w: 95, d: 60, r: 6 }, 3, 0.1, P)}" style="filter:blur(6px)"/>`);
  } else {
    for (const cx of PARKED) out.push(`<path class="iso-halo" d="${k.haloOf({ x: cx - 20, y: -34, w: 40, d: 68, r: 5 }, 0, 0.1, P)}" style="filter:blur(5px)"/>`);
    out.push(`<path class="iso-halo" d="${k.haloOf({ x: 74, y: -35, w: 44, d: 70, r: 6 }, 0, 0.1, P)}" style="filter:blur(6px)"/>`);
  }
  return out.join('');
}

/* ------------------------------------------------------------------ */
/* Car pose per scene: one rigid transform + throttle, pure in p.      */
/* ------------------------------------------------------------------ */

const ss = (a, b, x) => { const t = Math.min(1, Math.max(0, (x - a) / (b - a))); return t * t * (3 - 2 * t); };

// Piecewise storyboard progress: pose continuity, not measured flight time.
function carPose(scene, q) {
  const u = ss(0, 1, q);
  switch (scene) {
    case 1: return {...supportedPose(-23 + 113 * u), throttle:0};
    case 2: return {...supportedPose(90 + (HOLD_FRONT - 90) * ss(0, 0.55, q)), throttle:0};
    case 3: {
      const hold = supportedPose(HOLD_FRONT), C0 = applyM(hold.M, [0,0,10]);
      const C = [C0[0] + (-85-C0[0])*u, 0, C0[2] + (119-C0[2])*u];
      const beta = -115 - 65*ss(0.3,1,q);
      return poseAtCentre(rotY(beta),C,{beta,throttle:0.15*Math.sin(Math.PI*q)});
    }
    case 4: {
      const gam = 180*(1-u), r=mulR(rotZ(180),rotX(gam));
      return poseAtCentre(r,[-85-25*u,0,119-67*u],{gam,beta:0,throttle:0.45*Math.sin(Math.PI*q)});
    }
    case 5: return poseAtCentre(rotZ(180),[-110,0,52-36*u],{beta:0,throttle:0.35*Math.sin(Math.PI*q)});
    case 6: return {M:{r:IDR,t:[-75+27*u,0,PAD_Z+40*u]},beta:0,throttle:Math.sin(Math.PI*q/2)};
    case 7: {const gam=117*ss(0.55,1,q); return {M:{r:rotX(gam),t:[-48+108*u,0,PAD_Z+40+44*u]},beta:0,gam,throttle:1-0.55*u};}
    case 8: return {M:{r:rotX(117+126*u),t:[60+52*u,0,PAD_Z+84]},beta:0,gam:117+126*u,throttle:0.45};
    case 9: return {M:{r:rotX(243+117*u),t:[112+120*u,0,PAD_Z+84-44*ss(0.55,1,q)]},beta:0,gam:243+117*u,throttle:0.45};
    default: return {M:{r:IDR,t:[232+38*u,0,PAD_Z+40*(1-u)]},beta:0,throttle:0.45*(1-u)};
  }
}

/* ------------------------------------------------------------------ */
/* Readout (every string ends 示意，非实测)                              */
/* ------------------------------------------------------------------ */

function readoutOf(scene, q, pose) {
  const text = [
    t(`弯坡入口 · 俯仰 ${(-(pose.beta ?? 0)).toFixed(0)}°`, `Curved-ramp entry · Pitch ${(-(pose.beta ?? 0)).toFixed(0)}°`),
    t(`过垂直附着 · 俯仰 ${(-(pose.beta ?? 0)).toFixed(0)}° · 吸附机构未画出`, `Past-vertical adhesion · Pitch ${(-(pose.beta ?? 0)).toFixed(0)}° · Suction mechanism omitted`),
    t('离坡脱离 · 从过垂直到倒悬', 'Ramp release · Past vertical to inverted'),
    t(`绕车身纵轴回正 · 滚转 ${(180-(pose.gam ?? 180)).toFixed(0)}°`, `Longitudinal-axis recovery · Roll ${(180-(pose.gam ?? 180)).toFixed(0)}°`),
    t('坡道演示着陆 · 四轮回到支撑面', 'Ramp-demo landing · Four wheels return to the support surface'),
    t('平地起跳 · 车身前移并升起', 'Level-ground takeoff · Forward motion and ascent'),
    t('越过五车横列 · 停车朝向与飞行方向垂直', 'Over five cars side by side · Parked heading perpendicular to flight'),
    t(`Semi 上空倒转 · 滚转 ${(pose.gam ?? 0).toFixed(0)}°`, `Inverted above Semi · Roll ${(pose.gam ?? 0).toFixed(0)}°`),
    t(`滚转回正 · 累计 ${(pose.gam ?? 180).toFixed(0)}°`, `Roll recovery · Accumulated ${(pose.gam ?? 180).toFixed(0)}°`),
    t('越过车列后着陆 · 四轮回到支撑面', 'Landing after the car row · Four wheels return to the support surface'),
  ][scene-1];
  return t(`${text} · 示意，非实测`, `${text} · Schematic, not measured`);
}

/* ------------------------------------------------------------------ */
/* Halo + jets                                                         */
/* ------------------------------------------------------------------ */

function carHaloSvg(scene, pose, P) {
  const [cx, cy, cz] = pose.M.t;
  const groundZ = scene <= 5 && cx >= -150 && cx <= -50 && cz < 40 ? DECK_Z + 0.12 : 0.15;
  const h = Math.max(0, cz - groundZ);
  const shrink = Math.min(1, h / 55);
  const r = 34 * (1 - 0.32 * shrink);
  const op = (0.5 * (1 - 0.5 * shrink)).toFixed(3);
  const ring = k.circleRing(cx, cy, r, 20);
  return `<path class="iso-halo" d="${k.onTop(ring, groundZ, P, true)}" style="filter:blur(4px);opacity:${op}"/>`;
}

// Restrained monochrome jet segments, smooth in p. No flames, no colour.
function jetsSvg(pose, q, P) {
  const th = pose.throttle ?? 0;
  if (th < 0.02) return '';
  const down = applyR(pose.M.r, [0, 0, -1]);
  const out = [];
  const span = [[0, 0.42, 'mid', 0.55], [0.42, 0.76, 'lo', 0.3], [0.76, 1, 'faint', 0.15]];
  NOZZLES.forEach((nz, i) => {
    const base = applyM(pose.M, [nz[0], nz[1], 0.7]);
    const len = th * (3 + 1.4 * Math.sin(2.3 * i + 7 * q));
    if (len < 0.05) return;
    for (const [f0, f1, tone, o0] of span) {
      const a = [base[0] + down[0] * len * f0, base[1] + down[1] * len * f0, base[2] + down[2] * len * f0];
      const b = [base[0] + down[0] * len * f1, base[1] + down[1] * len * f1, base[2] + down[2] * len * f1];
      out.push(`<g opacity="${(o0 * Math.min(1, th * 1.2)).toFixed(2)}">${k.lineSvg(k.segment(a, b, P), { tone })}</g>`);
    }
  });
  return out.join('');
}

/* ------------------------------------------------------------------ */
/* Scoped kit CSS: the Anatomy theme with every selector namespaced    */
/* under .k3iso, so the figure is self-contained and side-effect free. */
/* ------------------------------------------------------------------ */

const CSS = k.ISO_CSS.replace(/\.iso(?![a-zA-Z0-9_-])/g, '.k3iso') + `
.k3iso:not([data-theme="light"]){
  --anatomy-paper:#1b2228;--anatomy-face:#61717a;--anatomy-top:#59666c;
  --anatomy-shade-0:#283137;--anatomy-shade-1:#344047;--anatomy-shade-2:#445159;--anatomy-shade-3:#526169;
  --anatomy-lit-top:#6b7b84;--anatomy-lit-shade:#526169;
  --anatomy-hi:#e2e5e8;--anatomy-mid:#91a0a9;--anatomy-lo:#657781;--anatomy-faint:#43535d;
}
.k3iso [data-material="paint"]{--anatomy-paper:#b7c0c5;--anatomy-top:#dce0e2;--anatomy-shade-0:#77858d;--anatomy-shade-1:#98a5ac;--anatomy-shade-2:#b8c1c6;--anatomy-shade-3:#cbd2d6;--anatomy-hi:#e8edef;--anatomy-faint:#778890}
.k3iso [data-material="glass"]{--anatomy-paper:#131b20;--anatomy-top:#253034;--anatomy-shade-0:#0d1418;--anatomy-shade-1:#172126;--anatomy-shade-2:#202b30;--anatomy-shade-3:#303b40;--anatomy-hi:#68747b;--anatomy-faint:#27353d}
.k3iso [data-material="rubber"],.k3iso [data-material="void"]{--anatomy-paper:#101418;--anatomy-top:#20262a;--anatomy-shade-0:#0c1013;--anatomy-shade-1:#13191e;--anatomy-shade-2:#1b2228;--anatomy-shade-3:#252d33;--anatomy-hi:#414d55;--anatomy-lo:#46525b;--anatomy-faint:#26343b}
.k3iso [data-material="carbon"]{--anatomy-paper:#202b32;--anatomy-top:#35444d;--anatomy-shade-0:#18232a;--anatomy-shade-1:#24323b;--anatomy-shade-2:#30414b;--anatomy-shade-3:#3d4f59;--anatomy-hi:#7d919d;--anatomy-lo:#748992;--anatomy-faint:#40535d}
.k3iso [data-material="metal"]{--anatomy-paper:#9aa8b0;--anatomy-top:#c3cdd2;--anatomy-shade-0:#5e727e;--anatomy-shade-1:#7e909a;--anatomy-shade-2:#a1afb7;--anatomy-shade-3:#b4c1c9;--anatomy-hi:#d5dfe4;--anatomy-faint:#718791}
.k3iso[data-theme="light"] [data-material="paint"]{--anatomy-paper:#d5dde1;--anatomy-top:#fafcfc;--anatomy-shade-0:#9eafb9;--anatomy-shade-1:#bcc8cf;--anatomy-shade-2:#d7dfe4;--anatomy-shade-3:#edf0f2;--anatomy-hi:#566973;--anatomy-faint:#a0b0b9}
.k3iso[data-theme="light"] [data-material="metal"]{--anatomy-paper:#bec9cf;--anatomy-top:#e1e6e9;--anatomy-shade-0:#8b9ea9;--anatomy-shade-1:#a2b3bd;--anatomy-shade-2:#bfccd4;--anatomy-shade-3:#d3dde3;--anatomy-hi:#506976;--anatomy-faint:#8eabb9}
`;

/* ------------------------------------------------------------------ */
/* Scene assembly                                                      */
/* ------------------------------------------------------------------ */

const LABELS = {
  1: t('场景1 弯坡入口：低趴跑车模型沿平板驶向分段弯坡并上行，示意模型，非实测', 'Scene 1, curved-ramp entry: a low sports-car model drives along the flat deck and up a segmented curved ramp; schematic model, not measured'),
  2: t('场景2 倒悬吸附：跑车模型贴合在过垂直斜坡上，吸附结构未画出，示意模型，非实测', 'Scene 2, past-vertical adhesion: the model contacts a past-vertical ramp; suction hardware is omitted; schematic model, not measured'),
  3: t('场景3 倒悬脱离：跑车模型离开过垂直斜坡并逐步转到倒悬，示意模型，非实测', 'Scene 3, ramp release: the model leaves the past-vertical ramp and gradually turns inverted; schematic model, not measured'),
  4: t('场景4 空中180°回正：跑车模型在空中绕车身纵轴滚转半周恢复轮朝下，示意模型，非实测', 'Scene 4, airborne 180° recovery: the model rolls half a turn about its longitudinal axis to restore wheels-down attitude; schematic model, not measured'),
  5: t('场景5 坡道演示着陆：跑车模型降落在坡道旁平台上的标线区，示意模型，非实测', 'Scene 5, ramp-demo landing: the model lands in the marked zone on a platform beside the ramp; schematic model, not measured'),
  6: t('场景6 无坡起跳：跑车模型从平地垫面前移并升起，示意模型，非实测', 'Scene 6, level-ground takeoff: the model moves forward and rises from a flat pad; schematic model, not measured'),
  7: t('场景7 爬升越过车列：跑车模型爬升越过五车并排的参照车列，车列与飞行方向垂直，示意模型，非实测', 'Scene 7, climb over five cars: the model climbs over five reference cars parked side by side perpendicular to flight; schematic model, not measured'),
  8: t('场景8 Semi上空倒转：跑车模型在Semi形参照卡车上方滚转至倒悬，示意模型，非实测', 'Scene 8, inverted above Semi: the model rolls inverted above a Semi-shaped reference truck; schematic model, not measured'),
  9: t('场景9 滚转回正：跑车模型滚转半周恢复正飞，示意模型，非实测', 'Scene 9, roll recovery: the model rolls half a turn back to upright flight; schematic model, not measured'),
  10: t('场景10 车群演示着陆：跑车模型越过车列后降落在着陆垫上，示意模型，非实测', 'Scene 10, landing after the car row: the model lands on a pad after clearing the cars; schematic model, not measured'),
};

const ENV_CACHE = new Map();
function svgOf(scene, q) {
  const P = projFor(scene);
  const pose = carPose(scene, q);
  if (!ENV_CACHE.has(scene)) ENV_CACHE.set(scene, {bench: benchSvg(scene, P), items: envParts(scene).map(d => buildItem(d, d.m ?? {r: IDR, t:[0,0,0]}, P))});
  const env = ENV_CACHE.get(scene);
  const items = orderItems([...env.items, ...CAR_DEFS.map(d => buildItem(d, pose.M, P))]);
  const body = [
    env.bench,
    carHaloSvg(scene, pose, P),
    items.map((it) => it.svg).join(''),
    jetsSvg(pose, q, P),
  ].join('');
  return `<svg xmlns="http://www.w3.org/2000/svg" class="iso-svg" viewBox="0 0 ${VW} ${VH}" role="img" aria-label="${LABELS[scene]}"><style>${CSS}</style>${body}</svg>`;
}

const clamp01 = (p) => {
  const v = Number(p);
  if (!Number.isFinite(v)) return 0.5;
  return Math.min(1, Math.max(0, v));
};

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

export function mount(container, { scene = 1, theme = 'dark', onReadout = () => {} } = {}) {
  const sc = Math.min(10, Math.max(1, Math.round(Number(scene) || 1)));
  const wrap = document.createElement('div');
  wrap.className = 'k3iso iso';
  wrap.dataset.theme = theme === 'light' ? 'light' : 'dark';
  container.appendChild(wrap);
  const render = (p) => {
    const q = clamp01(p);
    wrap.innerHTML = svgOf(sc, q);
    onReadout(readoutOf(sc, q, carPose(sc, q)));
  };
  render(0.5);
  return {
    setProgress: render,
    destroy() { wrap.remove(); },
  };
}

/* Test hooks (pure model only; mount() itself needs a DOM). */
export const __test = {
  clamp01, svgOf, readoutOf, carPose, buildItem, orderItems,
  carDefs: () => CAR_DEFS, envParts,
  carWorldVerts: (scene, q) => { const M = carPose(scene, q).M; return CAR_DEFS.flatMap((d) => d.geo.v.map((v) => applyM(M, v))); },
  applyM, supportedPose, wheelPath, HOLD_FRONT, ARC_LENGTH, TRACK_END, CSS, LABELS, PAD_Z, projFor, LOOP, RO, DECK_Z, PARKED, PARKED_HALF, PARKED_ROOF, PARKED_YHALF, SEMI,
  CAR_AXLE_HALF, CAR_TIRE_R,
  partCount: (scene) => envParts(scene).length + CAR_DEFS.length,
};
