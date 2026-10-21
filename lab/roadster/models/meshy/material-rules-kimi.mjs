export const metadata = {
  author: 'Kimi k3',
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
    'Material boundaries are per-triangle geometric approximations; positions and faces are untouched.',
    'Glass is opaque and dark because the continuous source contains no modeled interior.'
  ]
};

export const palette = {
  paint: { color: '#c5cbd1', metalness: 0.78, roughness: 0.32, clearcoat: 0.65, clearcoatRoughness: 0.18 },
  windshieldRoof: { color: '#10181e', metalness: 0.12, roughness: 0.17, clearcoat: 0.90, clearcoatRoughness: 0.08 },
  sideGlass: { color: '#18242b', metalness: 0.10, roughness: 0.21, clearcoat: 0.82, clearcoatRoughness: 0.10 },
  tire: { color: '#111214', metalness: 0.00, roughness: 0.90, clearcoat: 0.00, clearcoatRoughness: 1.00 },
  rim: { color: '#4b525a', metalness: 0.84, roughness: 0.38, clearcoat: 0.25, clearcoatRoughness: 0.24 },
  lowerTrim: { color: '#15191d', metalness: 0.20, roughness: 0.60, clearcoat: 0.12, clearcoatRoughness: 0.45 },
  headlight: { color: '#e8f3ff', metalness: 0.10, roughness: 0.22, emissive: '#dceeff', emissiveIntensity: 0.70 },
  taillight: { color: '#a91628', metalness: 0.08, roughness: 0.25, emissive: '#ff2940', emissiveIntensity: 0.45 }
};

export const regions = {
  wheels: {
    front: { x: -0.415, y: 0.008 },
    rear: { x: 0.500, y: 0.018 },
    outerRadius: 0.120,
    rimRadius: 0.094,
    minAbsZ: 0.170,
    outerSideAbsZ: 0.308,
    treadMaxAbsNormalZ: 0.78
  },
  glass: {
    frontX: -0.350,
    rearX: 0.420,
    minY: 0.152,
    roofMaxAbsZ: 0.238,
    sideFrontX: -0.200,
    sideRearX: 0.380,
    sideMinY: 0.125,
    sideMaxY: 0.230,
    sideMinAbsZ: 0.232,
    sideMaxAbsZ: 0.286,
    sideNormalMin: 0.22,
    pillarBands: [[-0.015, 0.020], [0.205, 0.240]]
  },
  trim: {
    underbodyY: -0.055,
    sillY: -0.034,
    sillMinAbsZ: 0.200,
    frontX: -0.540,
    frontMaxY: 0.042,
    rearX: 0.600,
    rearMaxY: 0.128,
    endNormalMin: 0.18
  },
  lights: {
    frontMinX: -0.625,
    frontMaxX: -0.515,
    frontMinY: 0.058,
    frontMaxY: 0.096,
    frontMinAbsZ: 0.145,
    frontMaxAbsZ: 0.295,
    frontNormalMax: -0.36,
    rearMinX: 0.665,
    rearMinY: 0.136,
    rearMaxY: 0.166,
    rearMinAbsZ: 0.100,
    rearMaxAbsZ: 0.305,
    rearNormalMin: 0.45
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

function wheelKey(x, y, az, absNz) {
  const w = regions.wheels;
  const c = x < 0 ? w.front : w.rear;
  const radius = Math.hypot(x - c.x, y - c.y);
  if (radius > w.outerRadius || az < w.minAbsZ) return null;
  if (radius < w.rimRadius) return 'rim';
  if (az > w.outerSideAbsZ || absNz < w.treadMaxAbsNormalZ) return 'tire';
  return null;
}

export function classifyTriangle(centroid, normal, vertices) {
  const x = centroid[0], y = centroid[1], az = Math.abs(centroid[2]);
  const nx = normal[0], ny = normal[1], absNz = Math.abs(normal[2]);
  const wheel = wheelKey(x, y, az, absNz);
  if (wheel) return wheel;

  const l = regions.lights;
  if (x >= l.frontMinX && x <= l.frontMaxX && y >= l.frontMinY && y <= l.frontMaxY &&
      az >= l.frontMinAbsZ && az <= l.frontMaxAbsZ && nx < l.frontNormalMax) return 'headlight';
  if (x > l.rearMinX && y >= l.rearMinY && y <= l.rearMaxY &&
      az >= l.rearMinAbsZ && az <= l.rearMaxAbsZ && nx > l.rearNormalMin) return 'taillight';

  const t = regions.trim;
  if (x < t.frontX && y < t.frontMaxY && nx < -t.endNormalMin) return 'lowerTrim';
  if (x > t.rearX && y < t.rearMaxY && nx > t.endNormalMin) return 'lowerTrim';
  if (y < t.underbodyY && ny < 0.45) return 'lowerTrim';
  if (y < t.sillY && az > t.sillMinAbsZ && absNz > 0.15) return 'lowerTrim';

  const g = regions.glass;
  const pillar = g.pillarBands.some(([lo, hi]) => x >= lo && x <= hi);
  if (!pillar && x >= g.sideFrontX && x <= g.sideRearX && y >= g.sideMinY && y <= g.sideMaxY &&
      az >= g.sideMinAbsZ && az <= g.sideMaxAbsZ && absNz > g.sideNormalMin) return 'sideGlass';
  if (x >= g.frontX && x <= g.rearX && y > g.minY && az < g.roofMaxAbsZ && ny > -0.25) {
    return 'windshieldRoof';
  }
  return 'paint';
}

export function uvForPoint(position, region) {
  const s = atlas.swatches[region] || atlas.swatches.paint;
  return [(s.u0 + s.u1) * 0.5, (s.v0 + s.v1) * 0.5];
}
