export const metadata = {
  author: 'Codex review of Kimi k3 material rules',
  parentRulesSha256: '6789dca14a1aea620e708e4a40e751de372790c9ef8e446635a4cc22c01deb53',
  coordinates: 'X length, Y up, Z width; front -X',
  referenceObservations: [
    'Mesh preview: continuous wedge body, raised canopy, recessed side windows and integral wheels.',
    'Official side: silver doors and fenders, narrow dark glass, bright pillars and black sills.',
    'Official front-quarter: silver hood meets one dark canopy above a black lower fascia.',
    'Official front: compact white lamps sit at the outer corners rather than across the whole nose.',
    'Official rear: a thin red accent borders the dark lower fascia while the deck remains silver.'
  ],
  limitations: [
    'Lamps are restrained painted accents because the source has no separated lamp hardware.',
    'Material boundaries approximate the supplied Meshy surface; no measured production-car material segmentation is claimed.',
    'Glass is opaque and dark because the continuous source contains no modeled interior.'
  ]
};

export const palette = {
  paint: { color: '#bfc5cb', metalness: 0.68, roughness: 0.39, clearcoat: 0.42, clearcoatRoughness: 0.23 },
  windshieldRoof: { color: '#10181e', metalness: 0.12, roughness: 0.17, clearcoat: 0.90, clearcoatRoughness: 0.08 },
  sideGlass: { color: '#10181e', metalness: 0.12, roughness: 0.20, clearcoat: 0.82, clearcoatRoughness: 0.10 },
  tire: { color: '#111214', metalness: 0.00, roughness: 0.90, clearcoat: 0.00, clearcoatRoughness: 1.00 },
  rim: { color: '#4b525a', metalness: 0.84, roughness: 0.38, clearcoat: 0.25, clearcoatRoughness: 0.24 },
  lowerTrim: { color: '#20252a', metalness: 0.20, roughness: 0.60, clearcoat: 0.12, clearcoatRoughness: 0.45 },
  headlight: { color: '#e8f3ff', metalness: 0.10, roughness: 0.22, emissive: '#dceeff', emissiveIntensity: 0.30 },
  taillight: { color: '#a91628', metalness: 0.08, roughness: 0.25, emissive: '#ff2940', emissiveIntensity: 0.28 }
};

export const regions = {
  wheels: {
    front: { x: -0.415, y: 0.008 },
    rear: { x: 0.500, y: 0.018 },
    outerRadius: 0.109,
    rimRadius: 0.090,
    minAbsZ: 0.255,
    rimMinAbsZ: 0.282
  },
  glass: {
    frontX: -0.355,
    rearX: 0.380,
    // Outline follows the mesh's side-plane crease, leaving one narrow silver perimeter.
    sidePolygonXY: [
      [-0.285, 0.151], [-0.205, 0.193], [-0.105, 0.233], [-0.035, 0.256],
      [0.105, 0.258], [0.235, 0.249], [0.345, 0.235], [0.363, 0.224],
      [0.294, 0.210], [0.160, 0.191], [0.010, 0.175], [-0.145, 0.158]
    ],
    // X/half-width stations for a continuous windshield and roof; no artificial crossbars.
    roofWidths: [[-0.355, 0.247], [-0.24, 0.242], [-0.10, 0.218], [0.08, 0.213], [0.27, 0.216], [0.38, 0.223]],
    sideMinAbsZ: 0.215,
    sideMaxAbsZ: 0.288
  },
  trim: {
    underbodyY: -0.068,
    sillY: -0.052,
    sillMinAbsZ: 0.270,
    frontX: -0.590,
    frontMaxY: 0.025,
    frontHalfWidth: 0.229,
    rearX: 0.640,
    rearMaxY: 0.105,
    rearHalfWidth: 0.255
  },
  lights: {
    frontMinX: -0.641,
    frontMaxX: -0.535,
    frontCenterY: 0.084,
    frontHalfHeight: 0.004,
    frontMinAbsZ: 0.205,
    frontMaxAbsZ: 0.274,
    rearMinX: 0.685,
    rearCenterY: 0.154,
    rearHalfHeight: 0.0038,
    rearMaxAbsZ: 0.286
  }
};

const swatchOrder = ['paint', 'windshieldRoof', 'sideGlass', 'tire', 'rim', 'lowerTrim', 'headlight', 'taillight'];

export const atlas = {
  width: 256,
  height: 32,
  swatches: Object.fromEntries(swatchOrder.map((key, i) => [key, {
    u0: (i * 32 + 2) / 256,
    v0: 2 / 32,
    u1: ((i + 1) * 32 - 2) / 256,
    v1: 30 / 32
  }]))
};

function wheelKey(x, y, az) {
  const w = regions.wheels;
  const c = x < 0 ? w.front : w.rear;
  const radius = Math.hypot(x - c.x, y - c.y);
  if (radius > w.outerRadius || az < w.minAbsZ) return null;
  if (radius < w.rimRadius && az > w.rimMinAbsZ) return 'rim';
  return 'tire';
}

function inPolygon(x, y, points) {
  let inside = false;
  for (let i = 0, j = points.length - 1; i < points.length; j = i++) {
    const [xi, yi] = points[i], [xj, yj] = points[j];
    if ((yi > y) !== (yj > y) && x < (xj - xi) * (y - yi) / (yj - yi) + xi) inside = !inside;
  }
  return inside;
}

function interpolate(stations, x) {
  for (let i = 1; i < stations.length; i++) if (x <= stations[i][0]) {
    const [a, b] = [stations[i - 1], stations[i]], t = Math.max(0, Math.min(1, (x - a[0]) / (b[0] - a[0])));
    return a[1] + t * (b[1] - a[1]);
  }
  return stations.at(-1)[1];
}

export function classifyTriangle(centroid, normal, vertices) {
  const x = centroid[0], y = centroid[1], az = Math.abs(centroid[2]);
  const nx = normal[0], ny = normal[1], absNz = Math.abs(normal[2]);
  const wheel = wheelKey(x, y, az);
  if (wheel) return wheel;

  const l = regions.lights;
  const frontLineY = l.frontCenterY + 0.04 * (az - 0.22);
  if (x >= l.frontMinX && x <= l.frontMaxX && Math.abs(y - frontLineY) < l.frontHalfHeight &&
      az >= l.frontMinAbsZ && az <= l.frontMaxAbsZ && nx < -0.12) return 'headlight';
  const rearLineY = l.rearCenterY + 0.005 * (1 - (az / l.rearMaxAbsZ) ** 2);
  if (x > l.rearMinX && Math.abs(y - rearLineY) < l.rearHalfHeight &&
      az < l.rearMaxAbsZ && nx > 0.10) return 'taillight';

  const t = regions.trim;
  if (x < t.frontX && y < t.frontMaxY && az < t.frontHalfWidth) return 'lowerTrim';
  if (x > t.rearX && y < t.rearMaxY && az < t.rearHalfWidth) return 'lowerTrim';
  if (y < t.underbodyY) return 'lowerTrim';
  if (x > -0.29 && x < 0.38 && y < t.sillY && az > t.sillMinAbsZ) return 'lowerTrim';

  const g = regions.glass;
  if (az >= g.sideMinAbsZ && az <= g.sideMaxAbsZ && inPolygon(x, y, g.sidePolygonXY)) return 'sideGlass';
  const frontBoundary = g.frontX + 0.014 * (az / 0.25) ** 2;
  if (x >= frontBoundary && x <= g.rearX && y > 0.145 && az < interpolate(g.roofWidths, x)) {
    return 'windshieldRoof';
  }
  return 'paint';
}

export function uvForPoint(position, region) {
  const s = atlas.swatches[region] || atlas.swatches.paint;
  return [(s.u0 + s.u1) * 0.5, (s.v0 + s.v1) * 0.5];
}
