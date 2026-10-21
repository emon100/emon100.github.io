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
/* Camera: one azimuth/elevation and one miniature scale for all ten   */
/* scenes; only the framing origin pans per scene group.               */
/* ------------------------------------------------------------------ */

const VW = 640, VH = 430, PAD = 30, AZ = 45, EL = 30;
const SINA = Math.sin(AZ * DEG), COSA = Math.cos(AZ * DEG), SINE = Math.sin(EL * DEG), COSE = Math.cos(EL * DEG);
const V3 = [COSA * COSE, SINA * COSE, SINE];        // unit view direction (toward viewer)
const L2 = [-SINA, COSA];                            // the kit's horizontal side light

const REGIONS = {
  A: [...k.boxCorners({ x: -165, y: -85, w: 210, d: 170 }, -17, 0), [-55, 0, 182]],
  B: [...k.boxCorners({ x: -80, y: -85, w: 310, d: 170 }, -17, 0), [75, 0, 92]],
};
const SCALE = Math.min(
  k.fitProjection(REGIONS.A, VW, VH, { pad: PAD, azimuth: AZ, elevation: EL }).scale,
  k.fitProjection(REGIONS.B, VW, VH, { pad: PAD, azimuth: AZ, elevation: EL }).scale,
);
const projFor = (scene) => {
  const pts = REGIONS[scene <= 5 ? 'A' : 'B'];
  const unit = { origin: [0, 0], scale: 1, azimuth: AZ, elevation: EL };
  const flat = pts.map((p) => k.iso(p, unit));
  const xs = flat.map((p) => p[0]), ys = flat.map((p) => p[1]);
  const l = Math.min(...xs), r = Math.max(...xs), u = Math.min(...ys), d = Math.max(...ys);
  return { origin: [VW / 2 - ((l + r) / 2) * SCALE, VH / 2 - ((u + d) / 2) * SCALE], scale: SCALE, azimuth: AZ, elevation: EL };
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
    if (dot3(nw, V3) <= 0.05) return '';
  }
  if (det.type === 'line') return k.lineSvg(k.pathOf(det.pts.map((p) => k.iso(applyM(M, p), P))), { tone: det.tone ?? 'faint' });
  if (det.type === 'dot') return k.dotsSvg([k.iso(applyM(M, det.at), P)], { size: det.size ?? 0.45, tone: det.tone ?? 'mid' });
  return '';
}

function buildItem(def, M, P) {
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
  const svg = `<g class="iso-solid"${def.lit ? ' data-lit' : ''}>${chunks.join('')}</g>`;
  return { svg, wv, planes, dmin, dmax, cd: k.depthOf(pc, P), bbox: [x0, y0, x1, y1] };
}

// Measured painter's order: centroid depth, then Newell plane tests.
const SEP_EPS = 0.02;
function orderItems(items) {
  items.sort((a, b) => a.cd - b.cd);
  const behind = (A, B) => {
    if (A.dmax <= B.dmin + SEP_EPS) return true;
    const [ax0, ay0, ax1, ay1] = A.bbox, [bx0, by0, bx1, by1] = B.bbox;
    if (ax0 > bx1 || bx0 > ax1 || ay0 > by1 || by0 > ay1) return true;
    for (const pl of B.planes) {
      if (dot3(pl.n, V3) <= 1e-6) continue;
      let ok = true;
      for (const v of A.wv) if (dot3(pl.n, v) > pl.d + SEP_EPS) { ok = false; break; }
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
/* The miniature vehicle. Local frame: x forward, z up from the tire   */
/* contact plane. Wheelbase 64 (axles at x = ±32), tires r 6.          */
/* Hypothetical layout — not a claim about any real vehicle.           */
/* ------------------------------------------------------------------ */

const TUB_RING = [
  [-37, -12], [-24, -18], [10, -18], [24, -14.5], [38, -8],
  [38, 8], [24, 14.5], [10, 18], [-24, 18], [-37, 12],
];
const HOOD_TOP = (x) => 13 - ((x - 24) / 14) * 2;   // sloped nose, x in 24..38

function carParts() {
  const P = [];
  const both = (fn) => { P.push(fn(1)); P.push(fn(-1)); };
  P.push({ name: 'pan', tone: 'lo', geo: box(-35, 35, -16, 16, 4.5, 7) });
  P.push({ name: 'tub', tone: 'hi', geo: extrudeAlong(TUB_RING, 'z', 7, 13), details: [
    { type: 'line', n: [0, 1, 0], pts: [[-2, 18.06, 7.3], [-2, 18.06, 12.7]] },
    { type: 'line', n: [0, -1, 0], pts: [[-2, -18.06, 7.3], [-2, -18.06, 12.7]] },
    { type: 'line', n: [0, 1, 0], pts: [[-22, 18.06, 12.85], [8, 18.06, 12.85]] },
    { type: 'line', n: [0, -1, 0], pts: [[-22, -18.06, 12.85], [8, -18.06, 12.85]] },
  ] });
  P.push({ name: 'hood', tone: 'hi', geo: wedgeGeo({ x0: 24, x1: 38, y0a: -14.5, y1a: 14.5, y0b: -8, y1b: 8, z0a: 7, z0b: 7, z1a: 13, z1b: 11 }), details: [
    { type: 'line', n: [1 / 7, 0, 1], pts: [[29, 4.5, HOOD_TOP(29) + 0.12], [34, 4.5, HOOD_TOP(34) + 0.12], [34, 7.5, HOOD_TOP(34) + 0.12], [29, 7.5, HOOD_TOP(29) + 0.12], [29, 4.5, HOOD_TOP(29) + 0.12]] },
    { type: 'line', n: [1 / 7, 0, 1], pts: [[29, -4.5, HOOD_TOP(29) + 0.12], [34, -4.5, HOOD_TOP(34) + 0.12], [34, -7.5, HOOD_TOP(34) + 0.12], [29, -7.5, HOOD_TOP(29) + 0.12], [29, -4.5, HOOD_TOP(29) + 0.12]] },
  ] });
  P.push({ name: 'nose', tone: 'hi', geo: wedgeGeo({ x0: 24, x1: 38, y0a: -16, y1a: 16, y0b: -8, y1b: 8, z0a: 7, z0b: 4.8, z1a: 7, z1b: 7 }) });
  P.push({ name: 'splitter', tone: 'hi', geo: box(26, 37, -14, 14, 3.6, 4.8) });
  both((s) => ({ name: `fence${s}`, tone: 'hi', geo: wedgeGeo({ x0: 30, x1: 36, y0a: s > 0 ? 12.5 : -14, y1a: s > 0 ? 14 : -12.5, y0b: s > 0 ? 12.5 : -14, y1b: s > 0 ? 14 : -12.5, z0a: 4.8, z0b: 4.8, z1a: 7 - (6 / 14) * 2.2, z1b: 7 - (12 / 14) * 2.2 }) }));
  P.push({ name: 'cockpit', tone: 'hi', geo: box(-12, 12, -11, 11, 13, 17.5) });
  P.push({ name: 'deck', tone: 'hi', geo: wedgeGeo({ x0: -37, x1: -12, y0a: -12, y1a: 12, y0b: -18, y1b: 18, z0a: 13, z0b: 13, z1a: 15, z1b: 15 }) });
  P.push({ name: 'lip', tone: 'hi', geo: box(-38, -32, -12.5, 12.5, 15, 16) });
  P.push({ name: 'diffuser', tone: 'hi', geo: wedgeGeo({ x0: -37, x1: -28, y0a: -12, y1a: 12, y0b: -12, y1b: 12, z0a: 3.5, z0b: 4.5, z1a: 4.5, z1b: 4.5 }) });
  P.push({ name: 'windscreen', tone: 'mid', geo: wedgeGeo({ x0: 5, x1: 12, y0a: -10, y1a: 10, y0b: -10, y1b: 10, z0a: 17.5, z0b: 17.5, z1a: 20.3, z1b: 17.5 }) });
  P.push({ name: 'roof', tone: 'mid', geo: box(-9, 5, -10, 10, 17.5, 20.3), details: [
    { type: 'line', n: [0, 0, 1], pts: [[-7, 8, 20.45], [3, 8, 20.45], [3, -8, 20.45], [-7, -8, 20.45], [-7, 8, 20.45]] },
  ] });
  both((s) => ({ name: `sill${s}`, tone: 'hi', geo: box(-22, 8, s > 0 ? 18 : -20.2, s > 0 ? 20.2 : -18, 4, 6.6) }));
  both((s) => ({ name: `intake${s}`, tone: 'lo', geo: box(-14, -3, s > 0 ? 18 : -19.3, s > 0 ? 19.3 : -18, 9, 12) }));
  both((s) => ({ name: `mstalk${s}`, tone: 'hi', geo: box(11, 13, s > 0 ? 17.7 : -19.5, s > 0 ? 19.5 : -17.7, 13, 14) }));
  both((s) => ({ name: `mhead${s}`, tone: 'hi', geo: box(12, 14.2, s > 0 ? 19.5 : -21.7, s > 0 ? 21.7 : -19.5, 13.2, 14.8) }));
  P.push({ name: 'taillight', tone: 'lo', geo: box(-38, -37, -8, 8, 10.5, 12) });
  for (const sx of [32, -32]) for (const sy of [1, -1]) {
    P.push({ name: `tire${sx}_${sy}`, tone: 'lo', geo: cylGeo('y', sx, 6, sy > 0 ? 13.5 : -19.5, sy > 0 ? 19.5 : -13.5, 6, 12) });
    P.push({ name: `hub${sx}_${sy}`, tone: 'hi', geo: cylGeo('y', sx, 6, sy > 0 ? 19.5 : -20.4, sy > 0 ? 20.4 : -19.5, 3.6, 12), details: [
      { type: 'dot', n: [0, sy, 0], at: [sx, sy * 20.5, 6], size: 0.5, tone: 'hi' },
    ] });
  }
  for (const sx of [30, -30]) for (const sy of [14.5, -14.5]) {
    P.push({ name: `pod${sx}_${sy}`, tone: 'hi', lit: true, geo: box(sx - 2.5, sx + 2.5, sy - 1.5, sy + 1.5, 2.5, 4.5) });
    P.push({ name: `nozzle${sx}_${sy}`, tone: 'hi', geo: cylGeo('z', sx, sy, 1.4, 2.5, 1.5, 8) });
    P.push({ name: `tip${sx}_${sy}`, tone: 'hi', geo: cylGeo('z', sx, sy, 0.7, 1.4, 0.9, 8) });
  }
  return P;
}
const CAR_DEFS = carParts();
const NOZZLES = [[30, 14.5], [30, -14.5], [-30, 14.5], [-30, -14.5]];

/* ------------------------------------------------------------------ */
/* Environment parts                                                   */
/* ------------------------------------------------------------------ */

const LOOP = { cx: -55, cz: 86, R: 80, band: 6 };
const RO = Math.sqrt(LOOP.R * LOOP.R - 32 * 32);   // origin radius on track (wheel tangent construction)
const DECK_Z = 6;

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
  // Segmented curved track: annular arc -90° → +90°, riding radius R.
  for (let i = 0; i < 10; i++) {
    const a0 = (-90 + i * 18) * DEG, a1 = a0 + 18 * DEG;
    const ring = [
      [LOOP.cx + LOOP.R * Math.cos(a0), LOOP.cz + LOOP.R * Math.sin(a0)],
      [LOOP.cx + LOOP.R * Math.cos(a1), LOOP.cz + LOOP.R * Math.sin(a1)],
      [LOOP.cx + (LOOP.R + 6) * Math.cos(a1), LOOP.cz + (LOOP.R + 6) * Math.sin(a1)],
      [LOOP.cx + (LOOP.R + 6) * Math.cos(a0), LOOP.cz + (LOOP.R + 6) * Math.sin(a0)],
    ];
    const am = (a0 + a1) / 2;
    const det = [{ type: 'dot', n: [0, 1, 0], at: [LOOP.cx + 83 * Math.cos(am), 28.15, LOOP.cz + 83 * Math.sin(am)], size: 0.5, tone: 'lo' }];
    // Face 3 = end at a1, face 5 = end at a0 (internal joints stay open).
    const skipFaces = [3, 5].filter((f) => !((i === 0 && f === 5) || (i === 9 && f === 3)));
    parts.push({ name: `track${i}`, tone: 'mid', geo: extrudeAlong(ring, 'y', -28, 28), skipFaces, details: det });
  }
  const zOut = (x) => LOOP.cz - Math.sqrt((LOOP.R + 6) ** 2 - (x - LOOP.cx) ** 2);
  parts.push({ name: 'post1', tone: 'mid', geo: wedgeGeo({ x0: -22, x1: -18, y0a: -3.5, y1a: 3.5, y0b: -3.5, y1b: 3.5, z0a: 0, z0b: 0, z1a: zOut(-22), z1b: zOut(-18) }) });
  parts.push({ name: 'post2', tone: 'mid', geo: wedgeGeo({ x0: -4, x1: 0, y0a: -3.5, y1a: 3.5, y0b: -3.5, y1b: 3.5, z0a: 0, z0b: 0, z1a: zOut(-4), z1b: zOut(0) }) });
  parts.push({ name: 'tie', tone: 'lo', geo: box(-22, 0, -3, 3, 0, 5) });
  parts.push({ name: 'tallpost', tone: 'mid', geo: box(-79, -71, 32, 40, 0, 172) });
  parts.push({ name: 'gusset', tone: 'lo', geo: box(-81, -69, 30, 42, 0, 5) });
  parts.push({ name: 'arm', tone: 'mid', geo: box(0, 17.9, -3, 3, -3, 3), m: { r: rotZ(-26.565), t: [-71, 36, 169] } });
  parts.push({ name: 'gateL', tone: 'mid', geo: box(-62, -58, 21, 27, 6, 52) });
  parts.push({ name: 'gateR', tone: 'mid', geo: box(-62, -58, -27, -21, 6, 52) });
  parts.push({ name: 'gateBar', tone: 'mid', geo: box(-63, -57, -27, 27, 52, 56) });
  parts.push({ name: 'puck', tone: 'mid', geo: cylGeo('z', -60, 0, 49, 52, 2.6, 16) });
  return parts;
}

const PARKED = [54, 94, 134];   // reference car centres on the lane
const PARKED_HALF = 16, PARKED_ROOF = 8.5, PARKED_YHALF = 9.3;
const SEMI = { x0: 52, x1: 148, roof: 30, yhalf: 13.5 };

function parkedCar(cx) {
  const m = { r: IDR, t: [cx, 0, 0] };
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
  return parts.map((p) => ({ ...p, m }));
}

function semiParts() {
  const parts = [
    { name: 'trailer', tone: 'mid', geo: box(78, 148, -12, 12, 10.5, 30), details: [
      { type: 'line', n: [0, 1, 0], pts: [[80, 12.15, 27], [146, 12.15, 27]], tone: 'faint' },
      ...[94, 110, 126].map((x) => ({ type: 'line', n: [0, 1, 0], pts: [[x, 12.15, 11.5], [x, 12.15, 29]], tone: 'faint' })),
      ...[84, 110, 142].map((x) => ({ type: 'dot', n: [0, 1, 0], at: [x, 12.15, 28.2], size: 0.45, tone: 'lo' })),
    ] },
    { name: 'gear', tone: 'lo', geo: box(82, 86, -4, 4, 0, 10.5) },
    { name: 'cab', tone: 'mid', geo: box(52, 70, -12, 12, 9, 22) },
    { name: 'rail', tone: 'lo', geo: box(55, 70, -8, 8, 4, 9) },
    { name: 'nose', tone: 'mid', geo: wedgeGeo({ x0: 70, x1: 76, y0a: -10.5, y1a: 10.5, y0b: -10.5, y1b: 10.5, z0a: 6, z0b: 6, z1a: 14, z1b: 11 }) },
    { name: 'fairing', tone: 'mid', geo: wedgeGeo({ x0: 52, x1: 62, y0a: -10, y1a: 10, y0b: -10, y1b: 10, z0a: 22, z0b: 22, z1a: 26, z1b: 22 }) },
    { name: 'glass', tone: 'lo', geo: wedgeGeo({ x0: 66, x1: 70, y0a: -11, y1a: 11, y0b: -11, y1b: 11, z0a: 15, z0b: 15, z1a: 21, z1b: 16.5 }) },
  ];
  for (const wx of [63, 132, 140]) for (const sy of [1, -1]) {
    parts.push({ name: 'semi.tire', tone: 'lo', geo: cylGeo('y', wx, 5, sy > 0 ? 9.5 : -13.5, sy > 0 ? 13.5 : -9.5, 5, 12), details: [
      { type: 'dot', n: [0, sy, 0], at: [wx, sy * 13.6, 5], size: 0.45, tone: 'mid' },
    ] });
  }
  return parts;
}

function laneParts(scene) {
  const parts = [];
  const pad = (x0, x1) => ({ name: 'pad', tone: 'lo', geo: box(x0, x1, -18, 18, 0, 0.8), details: [
    { type: 'line', n: [0, 0, 1], pts: [[x0 + 4, 14, 0.95], [x1 - 4, 14, 0.95], [x1 - 4, -14, 0.95], [x0 + 4, -14, 0.95], [x0 + 4, 14, 0.95]], tone: 'faint' },
    ...[[x0 + 4, 14], [x0 + 4, -14], [x1 - 4, 14], [x1 - 4, -14]].map(([x, y]) => ({ type: 'dot', n: [0, 0, 1], at: [x, y, 0.95], size: 0.5, tone: 'mid' })),
  ] });
  parts.push(pad(-58, -22));           // takeoff pad, present on the whole lane
  parts.push(pad(148, 226));           // landing pad
  if (scene === 7 || scene === 10) for (const cx of PARKED) parts.push(...parkedCar(cx));
  if (scene === 8 || scene === 9) parts.push(...semiParts());
  return parts;
}

const envParts = (scene) => (scene <= 5 ? fixtureParts() : laneParts(scene));

/* ------------------------------------------------------------------ */
/* Bench (kit floor-standing solids, always below everything else)     */
/* ------------------------------------------------------------------ */

function benchSvg(scene, P) {
  const isA = scene <= 5;
  const PLATE = isA ? { x: -165, y: -85, w: 210, d: 170, r: 8 } : { x: -80, y: -85, w: 310, d: 170, r: 8 };
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
  out.push(k.faceTextSvg(k.topMatrix([label.x + 17, label.y + 8.6], 0.8, P, 'x'), '示意', { size: 2.6, tone: 'lo', anchor: 'middle' }));
  // Static halos grounding the furniture.
  if (scene <= 5) {
    out.push(`<path class="iso-halo" d="${k.haloOf({ x: -83, y: 28, w: 18, d: 18, r: 4 }, 0, 0.1, P)}"/>`);
    out.push(`<path class="iso-halo" d="${k.haloOf({ x: -150, y: -30, w: 95, d: 60, r: 6 }, 3, 0.1, P)}" style="filter:blur(6px)"/>`);
  } else {
    if (scene === 7 || scene === 10) for (const cx of PARKED) out.push(`<path class="iso-halo" d="${k.haloOf({ x: cx - 18, y: -11, w: 36, d: 22, r: 5 }, 0, 0.1, P)}" style="filter:blur(5px)"/>`);
    if (scene === 8 || scene === 9) out.push(`<path class="iso-halo" d="${k.haloOf({ x: 50, y: -15, w: 128, d: 30, r: 6 }, 0, 0.1, P)}" style="filter:blur(6px)"/>`);
  }
  return out.join('');
}

/* ------------------------------------------------------------------ */
/* Car pose per scene: one rigid transform + throttle, pure in p.      */
/* ------------------------------------------------------------------ */

const ss = (a, b, x) => { const t = Math.min(1, Math.max(0, (x - a) / (b - a))); return t * t * (3 - 2 * t); };

// Pose riding the track at arc angle aDeg; gap > 0 pulls off the surface.
function trackPose(aDeg, gap = 0) {
  const a = aDeg * DEG, r = RO - gap;
  const t = [LOOP.cx + r * Math.cos(a), 0, LOOP.cz + r * Math.sin(a)];
  return { M: { r: rotY(-(aDeg + 90)), t }, beta: -(aDeg + 90), gap };
}

// Entry blend: front contact on the circle at angle phiDeg, rear contact
// still on the flat deck (z = DECK_Z). Two-point support, continuous pose.
function entryPose(phiDeg) {
  const phi = phiDeg * DEG;
  const F = [LOOP.cx + LOOP.R * Math.cos(phi), LOOP.cz + LOOP.R * Math.sin(phi)];
  const sB = Math.min(1, Math.max(-1, (F[1] - DECK_Z) / 64));
  const b = Math.asin(sB);                       // chord pitch above deck
  const Rp = [F[0] - 64 * Math.cos(b), F[1] - 64 * Math.sin(b)];
  return { M: { r: rotY(-b / DEG), t: [(F[0] + Rp[0]) / 2, 0, (F[1] + Rp[1]) / 2] }, beta: -b / DEG };
}

function carPose(scene, q) {
  switch (scene) {
    case 1: {
      if (q <= 0.55) return { M: { r: [...IDR], t: [-110 + 23 * (q / 0.55), 0, DECK_Z] }, beta: 0, throttle: 0 };
      if (q <= 0.8) return { ...entryPose(-90 + 47.13 * ((q - 0.55) / 0.25)), throttle: 0 };
      return { ...trackPose(-66.44 + 16.44 * ((q - 0.8) / 0.2)), throttle: 0 };
    }
    case 2: {
      const a = q <= 0.35 ? 55 + 35 * (q / 0.35) : 90;
      const gap = 0.35 * Math.exp(-9 * Math.max(0, q - 0.35));
      return { ...trackPose(a, gap), throttle: 0 };
    }
    case 3:
      return { M: { r: rotY(-180), t: [-55 - 3.2 * q, 0, LOOP.cz + RO - 16 * q * q] }, beta: -180, throttle: 0.6 * ss(0.7, 1, q) };
    case 4: {
      const x = -58.2 - 30 * q, z = LOOP.cz + RO - 16 - 105 * Math.pow(q, 1.35);
      const flip = ss(0, 0.87, q);
      const beta = -180 - 180 * flip;
      const throttle = q > 0.15 && q < 0.85 ? Math.sin((Math.PI * (q - 0.15)) / 0.7) : 0;
      return { M: { r: rotY(beta), t: [x, 0, z] }, beta, throttle };
    }
    case 5: {
      const x = -88.2 - 24.8 * q, z = 38.3156 - 32.3156 * (0.35 * q + 0.65 * q * q);
      const beta = 3 * (1 - q);
      return { M: { r: rotY(beta), t: [x, 0, z] }, beta, throttle: 0.5 * (1 - ss(0.5, 0.85, q)) };
    }
    case 6: {
      const u = Math.max(0, (q - 0.25) / 0.75);
      const z = 55 * Math.pow(u, 1.6);
      const beta = 4 * Math.sin(Math.PI * Math.min(1, u));
      return { M: { r: rotY(beta), t: [-40, 0, z] }, beta, throttle: ss(0.05, 0.35, q) };
    }
    case 7: {
      const x = -40 + 200 * q, z = 46 * (1 - (1 - q) * (1 - q));
      return { M: { r: [...IDR], t: [x, 0, z] }, beta: 0, throttle: 1 - 0.6 * ss(0.3, 0.9, q) };
    }
    case 8: case 9: {
      const gam = (scene === 8 ? 0 : 180) + 180 * q;
      const x = (scene === 8 ? 20 : 110) + (scene === 8 ? 90 : 35) * q;
      const ext = 19.5 * Math.abs(Math.sin(gam * DEG)) + Math.max(0, -26 * Math.cos(gam * DEG));
      const z = 35 + ext;
      return { M: { r: rotX(gam), t: [x, 0, z] }, beta: 0, gam, throttle: 0.45 + 0.35 * Math.sin(Math.PI * q) };
    }
    default: { // scene 10
      const x = 150 + 40 * q, z = 40 * (1 - q * q);
      const beta = -2 * (1 - q);
      return { M: { r: rotY(beta), t: [x, 0, z] }, beta, throttle: 0.5 * (1 - ss(0.5, 0.9, q)) };
    }
  }
}

/* ------------------------------------------------------------------ */
/* Readout (every string ends 示意，非实测)                              */
/* ------------------------------------------------------------------ */

function readoutOf(scene, q, pose) {
  const z = pose.M.t[2];
  switch (scene) {
    case 1: return `弯坡入口 · 姿态角 ${(-(pose.beta ?? 0)).toFixed(0)}° · 高度 ${(z - DECK_Z).toFixed(1)} · 示意，非实测`;
    case 2: return `倒悬吸附 · 附着机制不明 · 间隙 ${(pose.gap ?? 0).toFixed(2)} · 示意，非实测`;
    case 3: return `倒悬脱离 · 分离 ${(16 * q * q).toFixed(1)} · 示意，非实测`;
    case 4: return `空中180°回正 · 已转 ${(180 * ss(0, 0.87, q)).toFixed(0)}° · 高度 ${(z - DECK_Z).toFixed(1)} · 示意，非实测`;
    case 5: return `坡道演示着陆 · 高度 ${(z - DECK_Z).toFixed(1)} · 示意，非实测`;
    case 6: return `无坡起跳 · 高度 ${z.toFixed(1)} · 示意，非实测`;
    case 7: {
      const n = PARKED.filter((cx) => pose.M.t[0] > cx + PARKED_HALF).length;
      return `爬升越过车列 · 已过 ${n}/3 · 高度 ${z.toFixed(1)} · 示意，非实测`;
    }
    case 8: return `Semi上空倒转 · 滚转 ${(pose.gam ?? 0).toFixed(0)}° · 示意，非实测`;
    case 9: return `滚转回正 · 已转 ${((pose.gam ?? 180) - 180).toFixed(0)}° · 示意，非实测`;
    default: return `车群演示着陆 · 高度 ${z.toFixed(1)} · 示意，非实测`;
  }
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

function scopeCss(css) {
  let out = '', buf = '', inside = false;
  for (const c of css) {
    if (!inside) {
      if (c === '{') {
        const s = buf.trim();
        const keep = s.startsWith('@') || /^(to|from|\d+(?:\.\d+)?%)$/.test(s);
        out += keep ? `${buf}{` : `${s.split(',').map((x) => `.k3iso ${x.trim()}`).join(',')}{`;
        buf = ''; inside = true;
      } else buf += c;
    } else {
      if (c === '}') { out += `${buf}}`; buf = ''; inside = false; } else buf += c;
    }
  }
  return out + buf;
}
const CSS = scopeCss(k.ISO_CSS);

/* ------------------------------------------------------------------ */
/* Scene assembly                                                      */
/* ------------------------------------------------------------------ */

const LABELS = {
  1: '场景1 弯坡入口：低趴跑车模型沿平板驶向分段弯坡并上行，示意模型，非实测',
  2: '场景2 倒悬吸附：跑车模型倒悬贴合在弯坡顶端下侧，附着机制不明，示意模型，非实测',
  3: '场景3 倒悬脱离：跑车模型保持倒悬离开坡顶并出现间隙，示意模型，非实测',
  4: '场景4 空中180°回正：跑车模型在空中俯仰翻转半周恢复轮朝下，示意模型，非实测',
  5: '场景5 坡道演示着陆：跑车模型降落在坡道旁平台上的标线区，示意模型，非实测',
  6: '场景6 无坡起跳：跑车模型从平地垫面垂直升起，示意模型，非实测',
  7: '场景7 爬升越过车列：跑车模型爬升越过一排停放的参照小车，示意模型，非实测',
  8: '场景8 Semi上空倒转：跑车模型在Semi形参照卡车上方滚转至倒悬，示意模型，非实测',
  9: '场景9 滚转回正：跑车模型滚转半周恢复正飞，示意模型，非实测',
  10: '场景10 车群演示着陆：跑车模型越过车列后降落在着陆垫上，示意模型，非实测',
};

function svgOf(scene, q) {
  const P = projFor(scene);
  const pose = carPose(scene, q);
  const defs = [...envParts(scene).map((d) => ({ ...d, m: d.m ?? { r: IDR, t: [0, 0, 0] } })), ...CAR_DEFS.map((d) => ({ ...d, m: pose.M }))];
  const items = orderItems(defs.map((d) => buildItem(d, d.m, P)));
  const body = [
    benchSvg(scene, P),
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
  wrap.className = 'k3iso';
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
  applyM, LOOP, RO, DECK_Z, PARKED, PARKED_HALF, PARKED_ROOF, PARKED_YHALF, SEMI,
  partCount: (scene) => envParts(scene).length + CAR_DEFS.length,
};
