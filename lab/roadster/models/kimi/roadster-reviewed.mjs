export const metadata = {
  title: "Silver wedge Roadster — Codex-reviewed Kimi candidate",
  derivedFrom: "Kimi k3/max image-based source d220855d263b535006e7e2e3433803a0e5c6f65b5f9540873935ecdf6efe3f58",
  reviewAuthor: "Codex",
  reviewChanges: ["Monotone cubic longitudinal profiles remove artificial flat spots at every knot.", "Glazing and the underlying body share the same side-surface shape, preventing paint breaking through glass.", "Canopy extends forward toward the photo silhouette; hood is shorter and flatter.", "Larger wheel rims and thinner tire sidewalls match the source proportions more closely."],
  method: "Codex geometric review of the preserved independent Kimi model against the same four supplied photographs. This is a derived candidate, not a new independent Kimi output.",
  dimensionStatus: "COMMON COMPARISON ASSUMPTION, not verified factory dimensions and not a photogrammetric measurement.",
  assumedDimensionsMeters: { length: 4.10, width: 1.72, height: 1.20 },
  coordinateSystem: { front: "+X", left: "+Y", up: "+Z", ground: "z=0", origin: "vehicle midpoint on tire contact plane" },
  visuallyChosenProportionsMeters: {
    wheelbase: 2.62,
    frontAxleX: 1.30,
    rearAxleX: -1.32,
    wheelCenterTrack: 1.46,
    tireDiameter: 0.71,
    note: "These are manual modelling choices within the assumed envelope, inferred visually rather than measured."
  },
  sourceImageObservations: [
    {
      file: "Roadster-02-Pilot-Desktop.jpg",
      observations: "Long low silver wedge; short nose and forward front axle; roughly 2.6 m visual wheelbase at the assumed scale; strongly raked glass nose, arcing roof, pointed rear side glass, high rear haunches, darker lower door and black sill."
    },
    {
      file: "Roadster-08-End-Desktop.jpg",
      observations: "Broad low hood between raised fender shoulders; continuous dark windshield/roof; angular front cheek transitions; thin closed front light line; flush dark multi-spoke wheels and deep open arches."
    },
    {
      file: "Roadster-03-Carousel-Slide-1-Desktop.jpg",
      observations: "Frontal width is carried by the outer fenders; central nose is narrower; triangular outer lamp pockets sit above a thin central line and a deep black lower intake."
    },
    {
      file: "Roadster-03-Carousel-Slide-2-Desktop.jpg",
      observations: "Clean angular silver rear deck, thin muted red tail strip, black lower fascia, metallic-rimmed circular recess/outlet details and a dark low diffuser."
    }
  ],
  chosenState: "Predominantly CLOSED/STOWED body: Pilot and End master the silhouette; Slide-2 masters the rear; Slide-1 constrains frontal width and pocket placement only.",
  lampStateNote: "The model uses the closed thin white front line with subtle unlit smoked triangular pockets. This is not represented as a confirmed production lamp-state combination.",
  geometry: {
    body: "Codex-reviewed shell with monotone cubic longitudinal profiles, a flatter hood and retained real wheel-arch openings.",
    wheelWells: "Open exterior arches, separate curved dark well barrels, deep inner liners and silver tubular fender lips.",
    glazing: "Continuous opaque dark PBR windshield/roof/side-glass surface with tapered side glass and separate silver rails and pillars.",
    intendedTriangleBudget: "Below 150,000 triangles; actual count and bounds are computed into group.userData at creation."
  },
  limitations: [
    "Manually approximated contours and proportions; no precision or photogrammetric claim.",
    "Unseen underbody, wheel-well depth and small shut lines are simplified.",
    "Rear outlets are visible recess geometry only; no engine, pressurised plumbing or other unseen mechanism is inferred.",
    "No cabin interior is modeled; dark glazing is opaque so no HDRI or transparency sorting is required."
  ]
};

export function createRoadsterModel(THREE) {
  const group = new THREE.Group();
  group.name = "Roadster_Mother_Model";

  const clamp = (v, lo = 0, hi = 1) => Math.max(lo, Math.min(hi, v));
  const lerp = (a, b, t) => a + (b - a) * t;
  const smooth = t => {
    const u = clamp(t);
    return u * u * (3 - 2 * u);
  };
  const slopeCache = new WeakMap();
  const keyed = (keys, x) => {
    if (x <= keys[0][0]) return keys[0][1];
    if (x >= keys[keys.length - 1][0]) return keys[keys.length - 1][1];
    let slopes = slopeCache.get(keys);
    if (!slopes) {
      const h = keys.slice(1).map((p, i) => p[0] - keys[i][0]);
      const d = h.map((width, i) => (keys[i + 1][1] - keys[i][1]) / width);
      slopes = [d[0]];
      for (let i = 1; i < keys.length - 1; i++) {
        const w1 = 2 * h[i] + h[i - 1], w2 = h[i] + 2 * h[i - 1];
        slopes.push(d[i - 1] * d[i] <= 0 ? 0 : (w1 + w2) / (w1 / d[i - 1] + w2 / d[i]));
      }
      slopes.push(d[d.length - 1]);
      slopeCache.set(keys, slopes);
    }
    let i = 0;
    while (x > keys[i + 1][0]) i++;
    const a = keys[i], b = keys[i + 1], h = b[0] - a[0], t = (x - a[0]) / h;
    const t2 = t * t, t3 = t2 * t;
    return (2 * t3 - 3 * t2 + 1) * a[1] + (t3 - 2 * t2 + t) * h * slopes[i]
      + (-2 * t3 + 3 * t2) * b[1] + (t3 - t2) * h * slopes[i + 1];
  };

  const bodyMaterial = new THREE.MeshPhysicalMaterial({
    name: "Satin silver clearcoat",
    color: 0xb9bec4,
    metalness: 0.76,
    roughness: 0.31,
    clearcoat: 0.72,
    clearcoatRoughness: 0.20
  });
  const doorMaterial = new THREE.MeshPhysicalMaterial({
    name: "Darker lower door silver",
    color: 0x8d949c,
    metalness: 0.72,
    roughness: 0.37,
    clearcoat: 0.52,
    clearcoatRoughness: 0.25
  });
  const glassMaterial = new THREE.MeshPhysicalMaterial({
    name: "Rich dark opaque glazing",
    color: 0x090d13,
    metalness: 0.38,
    roughness: 0.16,
    clearcoat: 1.0,
    clearcoatRoughness: 0.07,
    side: THREE.DoubleSide
  });
  const aeroMaterial = new THREE.MeshStandardMaterial({
    name: "Black lower aero",
    color: 0x090b0e,
    metalness: 0.32,
    roughness: 0.48,
    side: THREE.DoubleSide
  });
  const wellMaterial = new THREE.MeshStandardMaterial({
    name: "Dark wheel-well interior",
    color: 0x050608,
    metalness: 0.05,
    roughness: 0.88,
    side: THREE.DoubleSide
  });
  const tireMaterial = new THREE.MeshStandardMaterial({
    name: "Near-black tire rubber",
    color: 0x111316,
    metalness: 0.03,
    roughness: 0.84
  });
  const rimMaterial = new THREE.MeshPhysicalMaterial({
    name: "Sculpted dark metallic wheels",
    color: 0x292e35,
    metalness: 0.90,
    roughness: 0.27,
    clearcoat: 0.28,
    clearcoatRoughness: 0.22,
    side: THREE.DoubleSide
  });
  const rimEdgeMaterial = new THREE.MeshStandardMaterial({
    name: "Machined dark rim edges",
    color: 0x505861,
    metalness: 0.94,
    roughness: 0.23
  });
  const brakeMaterial = new THREE.MeshStandardMaterial({
    name: "Brake discs",
    color: 0x555b60,
    metalness: 0.86,
    roughness: 0.43
  });
  const whiteLightMaterial = new THREE.MeshStandardMaterial({
    name: "Thin white front light",
    color: 0xeaf3ff,
    emissive: 0xdbeeff,
    emissiveIntensity: 1.7,
    metalness: 0.05,
    roughness: 0.24
  });
  const redLightMaterial = new THREE.MeshStandardMaterial({
    name: "Muted red rear light",
    color: 0xa32838,
    emissive: 0x690a17,
    emissiveIntensity: 0.85,
    metalness: 0.12,
    roughness: 0.29
  });
  const outletMaterial = new THREE.MeshStandardMaterial({
    name: "Rear outlet rims",
    color: 0x777b7d,
    metalness: 0.93,
    roughness: 0.30
  });

  const bodyGroup = new THREE.Group();
  bodyGroup.name = "body-and-fascia";
  const glazingGroup = new THREE.Group();
  glazingGroup.name = "glazing-and-pillars";
  const aeroGroup = new THREE.Group();
  aeroGroup.name = "black-sills-splitters-and-fascias";
  const lightGroup = new THREE.Group();
  lightGroup.name = "light-strips-and-unlit-pockets";
  const rearDetailGroup = new THREE.Group();
  rearDetailGroup.name = "rear-recess-and-outlet-details";
  group.add(bodyGroup, glazingGroup, aeroGroup, lightGroup, rearDetailGroup);

  const addMesh = (parent, name, geometry, material) => {
    const mesh = new THREE.Mesh(geometry, material);
    mesh.name = name;
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    parent.add(mesh);
    return mesh;
  };

  const buildGrid = (nx, ny, point, flip = false, materialSelector = null) => {
    const positions = [];
    for (let i = 0; i <= nx; i++) {
      for (let j = 0; j <= ny; j++) {
        const p = point(i, j);
        positions.push(p[0], p[1], p[2]);
      }
    }
    const primary = [];
    const secondary = [];
    for (let i = 0; i < nx; i++) {
      for (let j = 0; j < ny; j++) {
        const a = i * (ny + 1) + j;
        const b = (i + 1) * (ny + 1) + j;
        const c = (i + 1) * (ny + 1) + j + 1;
        const d = i * (ny + 1) + j + 1;
        const target = materialSelector && materialSelector(positions, a, b, c, d) === 1 ? secondary : primary;
        if (flip) target.push(a, c, b, a, d, c);
        else target.push(a, b, c, a, c, d);
      }
    }
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
    geometry.setIndex(primary.concat(secondary));
    if (materialSelector) {
      geometry.addGroup(0, primary.length, 0);
      if (secondary.length) geometry.addGroup(primary.length, secondary.length, 1);
    }
    geometry.computeVertexNormals();
    geometry.computeBoundingSphere();
    return geometry;
  };

  const FRONT = 2.02;
  const REAR = -2.02;
  const FRONT_AXLE = 1.30;
  const REAR_AXLE = -1.32;
  const AXLE_Z = 0.355;
  const TIRE_RADIUS = 0.355;
  const ARCH_RADIUS = 0.390;
  const TRACK_HALF = 0.73;
  const TIRE_HALF_WIDTH = 0.13;

  const widthKeys = [
    [-2.02, 0.70], [-1.90, 0.78], [-1.55, 0.825], [-1.32, 0.84],
    [-1.05, 0.825], [-0.55, 0.79], [0.0, 0.775], [0.55, 0.785],
    [1.05, 0.825], [1.30, 0.84], [1.60, 0.825], [1.90, 0.75], [2.02, 0.68]
  ];
  const baseKeys = [
    [-2.02, 0.24], [-1.75, 0.19], [-1.32, 0.18], [-0.70, 0.16],
    [0.0, 0.155], [0.70, 0.16], [1.30, 0.17], [1.75, 0.16], [2.02, 0.20]
  ];
  const sideTopKeys = [
    [-2.02, 0.88], [-1.80, 0.93], [-1.55, 0.95], [-1.32, 0.960],
    [-1.05, 0.95], [-0.65, 0.88], [-0.20, 0.835], [0.30, 0.825],
    [0.75, 0.810], [1.05, 0.815], [1.30, 0.815], [1.60, 0.76],
    [1.85, 0.64], [2.02, 0.54]
  ];
  const centerTopKeys = [
    [-2.02, 0.875], [-1.80, 0.90], [-1.55, 0.925], [-1.32, 0.94],
    [-1.25, 0.945], [-1.05, 1.015], [-0.65, 1.16], [-0.25, 1.192],
    [0.0, 1.190], [0.25, 1.145], [0.50, 1.035], [0.75, 0.905],
    [1.00, 0.850], [1.30, 0.7515], [1.60, 0.6530], [1.85, 0.5709], [2.02, 0.515]
  ];
  const canopyWidthKeys = [
    [-1.25, 0.30], [-1.10, 0.39], [-0.85, 0.45], [-0.45, 0.50],
    [0.0, 0.52], [0.35, 0.54], [0.60, 0.565], [0.80, 0.585], [1.00, 0.60]
  ];
  const glassBottomKeys = [
    [-1.25, 0.920], [-1.10, 0.91], [-0.85, 0.875], [-0.45, 0.835],
    [0.0, 0.825], [0.35, 0.830], [0.60, 0.835], [0.80, 0.815], [1.00, 0.825]
  ];
  const glassSpreadKeys = [
    [-1.25, 0.0], [-1.10, 0.055], [-0.85, 0.10], [-0.45, 0.13],
    [0.0, 0.14], [0.35, 0.13], [0.60, 0.08], [0.80, 0.04], [1.00, 0.0]
  ];

  const halfWidth = x => keyed(widthKeys, x);
  const baseHeight = x => keyed(baseKeys, x);
  const sideTop = x => keyed(sideTopKeys, x);
  const centerTop = x => keyed(centerTopKeys, x);
  const canopyHalf = x => keyed(canopyWidthKeys, x);
  const glassBottom = x => keyed(glassBottomKeys, x);
  const glassSpread = x => keyed(glassSpreadKeys, x);
  const inCanopy = x => x >= -1.25 && x <= 1.00;

  const archHeight = (x, axle) => {
    const dx = x - axle;
    if (Math.abs(dx) >= ARCH_RADIUS) return -Infinity;
    return AXLE_Z + Math.sqrt(Math.max(0, ARCH_RADIUS * ARCH_RADIUS - dx * dx));
  };
  const lowerSideHeight = x => Math.max(
    baseHeight(x),
    archHeight(x, FRONT_AXLE),
    archHeight(x, REAR_AXLE)
  );
  const sideY = (x, z) => {
    const t = clamp((z - baseHeight(x)) / Math.max(0.05, sideTop(x) - baseHeight(x)));
    const inset = 0.052 * Math.pow(1 - t, 1.25);
    return halfWidth(x) - inset;
  };
  const topHeight = (x, y) => {
    const a = Math.abs(y);
    const hw = halfWidth(x);
    const center = centerTop(x);
    const side = sideTop(x);
    if (inCanopy(x)) {
      const gh = canopyHalf(x);
      const gy = gh + glassSpread(x);
      const edge = center - 0.025;
      const bottom = glassBottom(x);
      if (a <= gh) return center - 0.025 * Math.pow(a / gh, 2);
      if (gy > gh + 1e-5 && a <= gy) return lerp(edge, bottom, smooth((a - gh) / (gy - gh)));
      return lerp(bottom, side, smooth((a - gy) / Math.max(0.02, hw - gy)));
    }
    if (x > 1.00) {
      const hoodHalf = hw * 0.55;
      if (a <= hoodHalf) return center - 0.003 * (a / hoodHalf) ** 2;
      return lerp(center - 0.003, side, (a - hoodHalf) / (hw - hoodHalf));
    }
    return lerp(center, side, smooth(a / hw));
  };

  let bodyXs = [];
  for (let i = 0; i <= 220; i++) bodyXs.push(lerp(REAR, FRONT, i / 220));
  for (const axle of [FRONT_AXLE, REAR_AXLE]) {
    for (let i = 0; i <= 64; i++) {
      const theta = Math.PI * i / 64;
      bodyXs.push(axle + ARCH_RADIUS * Math.cos(theta));
    }
    bodyXs.push(axle - ARCH_RADIUS - 0.0007, axle + ARCH_RADIUS + 0.0007);
  }
  bodyXs = bodyXs
    .filter(x => x >= REAR && x <= FRONT)
    .sort((a, b) => a - b)
    .filter((x, i, all) => i === 0 || Math.abs(x - all[i - 1]) > 1e-6);

  const BODY_SIDE_SEGMENTS = 14;
  const BODY_TOP_SEGMENTS = 48;
  const BODY_CROSS_SEGMENTS = BODY_SIDE_SEGMENTS * 2 + BODY_TOP_SEGMENTS;
  const bodyGeometry = buildGrid(
    bodyXs.length - 1,
    BODY_CROSS_SEGMENTS,
    (i, j) => {
      const x = bodyXs[i];
      const low = lowerSideHeight(x);
      const high = sideTop(x);
      if (j < BODY_SIDE_SEGMENTS) {
        const z = lerp(low, high, j / BODY_SIDE_SEGMENTS);
        return [x, sideY(x, z), z];
      }
      if (j <= BODY_SIDE_SEGMENTS + BODY_TOP_SEGMENTS) {
        const t = (j - BODY_SIDE_SEGMENTS) / BODY_TOP_SEGMENTS;
        const y = lerp(halfWidth(x), -halfWidth(x), t);
        return [x, y, topHeight(x, y)];
      }
      const t = (j - BODY_SIDE_SEGMENTS - BODY_TOP_SEGMENTS) / BODY_SIDE_SEGMENTS;
      const z = lerp(high, low, t);
      return [x, -sideY(x, z), z];
    },
    true,
    (positions, a, b, c, d) => {
      const ids = [a, b, c, d];
      let x = 0;
      let y = 0;
      let z = 0;
      for (const id of ids) {
        x += positions[id * 3] * 0.25;
        y += positions[id * 3 + 1] * 0.25;
        z += positions[id * 3 + 2] * 0.25;
      }
      return x < 0.68 && x > -0.80 && Math.abs(y) > 0.62 && z < sideTop(x) - 0.045 ? 1 : 0;
    }
  );
  const shell = addMesh(bodyGroup, "car-shell", bodyGeometry, [bodyMaterial, doorMaterial]);
  shell.userData.openWheelArches = true;
  shell.userData.archRadiusMeters = ARCH_RADIUS;

  const frontNoseGeometry = buildGrid(40, 18, (i, j) => {
    const y = lerp(-halfWidth(FRONT), halfWidth(FRONT), i / 40);
    const z = lerp(0.18, topHeight(FRONT, y), j / 18);
    return [FRONT, y, z];
  });
  addMesh(bodyGroup, "front-silver-nose", frontNoseGeometry, bodyMaterial);

  const rearCapGeometry = buildGrid(40, 18, (i, j) => {
    const y = lerp(-halfWidth(REAR), halfWidth(REAR), i / 40);
    const z = lerp(0.20, topHeight(REAR, y), j / 18);
    return [REAR, y, z];
  }, true);
  addMesh(bodyGroup, "rear-silver-deck-cap", rearCapGeometry, bodyMaterial);

  const frontFasciaGeometry = buildGrid(32, 8, (i, j) => {
    const y = lerp(-0.665, 0.665, i / 32);
    const top = 0.345 + 0.025 * Math.pow(Math.abs(y) / 0.665, 1.5);
    return [2.028, y, lerp(0.135, top, j / 8)];
  });
  addMesh(aeroGroup, "front-fascia", frontFasciaGeometry, aeroMaterial);

  const rearFasciaGeometry = buildGrid(36, 12, (i, j) => {
    const y = lerp(-0.69, 0.69, i / 36);
    const top = 0.665 + 0.015 * Math.pow(Math.abs(y) / 0.69, 2);
    return [-2.035, y, lerp(0.155, top, j / 12)];
  }, true);
  addMesh(aeroGroup, "rear-fascia", rearFasciaGeometry, aeroMaterial);

  const frontSplitterGeometry = buildGrid(14, 30, (i, j) => {
    const t = i / 14;
    const x = lerp(1.81, 2.05, t);
    const width = lerp(0.69, 0.765, t);
    const y = lerp(-width, width, j / 30);
    return [x, y, lerp(0.125, 0.095, t) + 0.012 * Math.pow(Math.abs(y) / width, 2)];
  });
  addMesh(aeroGroup, "front-splitter", frontSplitterGeometry, aeroMaterial);

  const rearDiffuserGeometry = buildGrid(14, 28, (i, j) => {
    const t = i / 14;
    const x = lerp(-2.05, -1.80, t);
    const width = lerp(0.72, 0.65, t);
    const y = lerp(-width, width, j / 28);
    return [x, y, lerp(0.10, 0.145, t) + 0.012 * Math.pow(Math.abs(y) / width, 2)];
  }, true);
  addMesh(aeroGroup, "rear-diffuser", rearDiffuserGeometry, aeroMaterial);

  const underbodyGeometry = buildGrid(20, 12, (i, j) => {
    const x = lerp(-1.86, 1.84, i / 20);
    const y = lerp(-0.55, 0.55, j / 12);
    return [x, y, 0.135];
  }, true);
  addMesh(aeroGroup, "simplified-underbody", underbodyGeometry, aeroMaterial);

  for (const side of [-1, 1]) {
    const sideName = side > 0 ? "left" : "right";
    const sillGeometry = buildGrid(42, 5, (i, j) => {
      const x = lerp(-0.885, 0.865, i / 42);
      const z = lerp(baseHeight(x) - 0.012, 0.265, j / 5);
      return [x, side * (sideY(x, z) + 0.005), z];
    }, side > 0);
    addMesh(aeroGroup, `${sideName}-black-sill`, sillGeometry, aeroMaterial);
  }

  const tube = (parent, name, points, radius, material, radialSegments = 7) => {
    const vectors = points.map(p => new THREE.Vector3(p[0], p[1], p[2]));
    const curve = new THREE.CatmullRomCurve3(vectors, false, "centripetal");
    const geometry = new THREE.TubeGeometry(curve, Math.max(24, points.length * 3), radius, radialSegments, false);
    return addMesh(parent, name, geometry, material);
  };

  const makeWellLiner = (axle, side, name) => {
    const boundary = [];
    const frontX = axle + ARCH_RADIUS;
    const rearX = axle - ARCH_RADIUS;
    boundary.push([frontX, baseHeight(frontX)]);
    boundary.push([frontX, AXLE_Z]);
    for (let i = 1; i <= 48; i++) {
      const theta = Math.PI * i / 48;
      boundary.push([axle + ARCH_RADIUS * Math.cos(theta), AXLE_Z + ARCH_RADIUS * Math.sin(theta)]);
    }
    boundary.push([rearX, baseHeight(rearX)]);
    const positions = [axle, side * 0.535, AXLE_Z + 0.08];
    for (const p of boundary) positions.push(p[0], side * 0.535, p[1]);
    const indices = [];
    for (let i = 0; i < boundary.length - 1; i++) indices.push(0, i + 1, i + 2);
    indices.push(0, boundary.length, 1);
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
    geometry.setIndex(indices);
    geometry.computeVertexNormals();
    addMesh(bodyGroup, name, geometry, wellMaterial);
  };

  for (const axle of [FRONT_AXLE, REAR_AXLE]) {
    const axleName = axle === FRONT_AXLE ? "front" : "rear";
    for (const side of [-1, 1]) {
      const sideName = side > 0 ? "left" : "right";
      const path = [];
      const frontX = axle + ARCH_RADIUS;
      const rearX = axle - ARCH_RADIUS;
      path.push([frontX, baseHeight(frontX)]);
      path.push([frontX, AXLE_Z]);
      for (let i = 1; i <= 48; i++) {
        const theta = Math.PI * i / 48;
        path.push([
          axle + ARCH_RADIUS * Math.cos(theta),
          AXLE_Z + ARCH_RADIUS * Math.sin(theta)
        ]);
      }
      path.push([rearX, baseHeight(rearX)]);
      const wellGeometry = buildGrid(path.length - 1, 5, (i, j) => {
        const p = path[i];
        const outer = sideY(p[0], p[1]) + 0.002;
        const y = side * lerp(0.535, outer, j / 5);
        return [p[0], y, p[1]];
      }, side < 0);
      addMesh(bodyGroup, `${axleName}-${sideName}-wheel-well-barrel`, wellGeometry, wellMaterial);
      makeWellLiner(axle, side, `${axleName}-${sideName}-deep-wheel-well-liner`);
      const lipPoints = path.map(p => [p[0], side * (sideY(p[0], p[1]) + 0.006), p[1]]);
      tube(bodyGroup, `${axleName}-${sideName}-curved-fender-lip`, lipPoints, 0.005, bodyMaterial, 8);
    }
  }

  const GLASS_SIDE_SEGMENTS = 12;
  const GLASS_TOP_SEGMENTS = 48;
  const GLASS_CROSS_SEGMENTS = GLASS_SIDE_SEGMENTS * 2 + GLASS_TOP_SEGMENTS;
  const glassGeometry = buildGrid(104, GLASS_CROSS_SEGMENTS, (i, j) => {
    const x = lerp(-1.25, 1.00, i / 104);
    const gh = canopyHalf(x);
    const gy = gh + glassSpread(x);
    const edge = centerTop(x) - 0.025;
    const bottom = glassBottom(x);
    if (j < GLASS_SIDE_SEGMENTS) {
      const t = j / GLASS_SIDE_SEGMENTS;
      const y = lerp(gy, gh, t) + 0.004 * (1 - t);
      return [x, y, lerp(bottom, edge, smooth(t)) + 0.008];
    }
    if (j <= GLASS_SIDE_SEGMENTS + GLASS_TOP_SEGMENTS) {
      const t = (j - GLASS_SIDE_SEGMENTS) / GLASS_TOP_SEGMENTS;
      const y = lerp(gh, -gh, t);
      return [x, y, centerTop(x) - 0.025 * Math.pow(Math.abs(y) / gh, 2) + 0.008];
    }
    const t = (j - GLASS_SIDE_SEGMENTS - GLASS_TOP_SEGMENTS) / GLASS_SIDE_SEGMENTS;
    const y = -lerp(gh, gy, t) - 0.004 * t;
    return [x, y, lerp(edge, bottom, smooth(t)) + 0.008];
  }, true);
  const glazing = addMesh(glazingGroup, "continuous-dark-glazing", glassGeometry, glassMaterial);
  glazing.userData.construction = "windshield, roof and tapering side glass in one curved parametric surface";

  const glassSidePoint = (x, side, t, offset = 0.008) => {
    const gh = canopyHalf(x);
    const gy = gh + glassSpread(x);
    const y = side * (lerp(gy, gh, t) + offset);
    const z = lerp(glassBottom(x), centerTop(x) - 0.025, smooth(t)) + 0.014;
    return [x, y, z];
  };

  for (const side of [-1, 1]) {
    const sideName = side > 0 ? "left" : "right";
    const railPoints = [];
    for (let i = 0; i <= 36; i++) {
      const x = lerp(-1.17, 0.90, i / 36);
      railPoints.push([x, side * (canopyHalf(x) + 0.006), centerTop(x) - 0.013]);
    }
    tube(glazingGroup, `${sideName}-silver-roof-rail`, railPoints, 0.007, bodyMaterial, 8);

    const aPoints = [
      glassSidePoint(0.97, side, 0.02),
      glassSidePoint(0.80, side, 0.28),
      glassSidePoint(0.62, side, 0.58),
      glassSidePoint(0.43, side, 0.88),
      glassSidePoint(0.33, side, 1.0)
    ];
    tube(glazingGroup, `${sideName}-silver-a-pillar`, aPoints, 0.008, bodyMaterial, 8);

    const cPoints = [
      glassSidePoint(-0.66, side, 1.0),
      glassSidePoint(-0.84, side, 0.80),
      glassSidePoint(-1.03, side, 0.46),
      glassSidePoint(-1.22, side, 0.03)
    ];
    tube(glazingGroup, `${sideName}-silver-c-pillar`, cPoints, 0.009, bodyMaterial, 8);

    const divisionPoints = [
      glassSidePoint(-0.30, side, 0.05, 0.006),
      glassSidePoint(-0.32, side, 0.96, 0.006)
    ];
    tube(glazingGroup, `${sideName}-dark-window-division`, divisionPoints, 0.005, aeroMaterial, 6);
  }

  const frontLineGeometry = buildGrid(32, 1, (i, j) => {
    const y = lerp(-0.50, 0.50, i / 32);
    const center = 0.466 + 0.007 * (1 - Math.pow(y / 0.50, 2));
    return [2.034, y, center + (j - 0.5) * 0.010];
  });
  addMesh(lightGroup, "front-thin-white-light-line", frontLineGeometry, whiteLightMaterial);

  const rearLineGeometry = buildGrid(36, 1, (i, j) => {
    const y = lerp(-0.665, 0.665, i / 36);
    const center = 0.681 + 0.008 * Math.pow(y / 0.665, 2);
    return [-2.042, y, center + (j - 0.5) * 0.012];
  }, true);
  addMesh(lightGroup, "rear-muted-red-light-strip", rearLineGeometry, redLightMaterial);

  for (const side of [-1, 1]) {
    const yInner = side * 0.505;
    const yOuter = side * 0.645;
    const positions = [
      2.031, yInner, 0.462,
      2.031, yOuter, 0.477,
      2.031, side * 0.615, 0.515
    ];
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
    geometry.setIndex(side > 0 ? [0, 1, 2] : [0, 2, 1]);
    geometry.computeVertexNormals();
    addMesh(lightGroup, `${side > 0 ? "left" : "right"}-unlit-triangular-headlight-pocket`, geometry, glassMaterial);
  }

  for (let i = 0; i < 5; i++) {
    const y = lerp(-0.46, 0.46, i / 4);
    const recessGeometry = new THREE.CylinderGeometry(0.031, 0.036, 0.036, 32, 1, false);
    recessGeometry.rotateZ(Math.PI / 2);
    const recess = addMesh(rearDetailGroup, `rear-recess-${i + 1}`, recessGeometry, wellMaterial);
    recess.position.set(-2.025, y, 0.30);
    const rimGeometry = new THREE.TorusGeometry(0.034, 0.004, 7, 32);
    rimGeometry.rotateY(Math.PI / 2);
    const rim = addMesh(rearDetailGroup, `rear-outlet-metal-rim-${i + 1}`, rimGeometry, outletMaterial);
    rim.position.set(-2.043, y, 0.30);
  }

  const tireProfile = [
    [0.286, -0.075], [0.292, -0.105], [0.309, -0.124], [0.329, -0.130],
    [0.345, -0.120], [0.353, -0.095], [0.355, -0.055], [0.355, 0.0],
    [0.355, 0.055], [0.353, 0.095], [0.345, 0.120], [0.329, 0.130],
    [0.309, 0.124], [0.292, 0.105], [0.286, 0.075]
  ];

  const buildTireGeometry = () => {
    const radialSegments = 72;
    const positions = [];
    for (const profile of tireProfile) {
      for (let j = 0; j < radialSegments; j++) {
        const theta = Math.PI * 2 * j / radialSegments;
        positions.push(profile[0] * Math.cos(theta), profile[1], profile[0] * Math.sin(theta));
      }
    }
    const indices = [];
    for (let i = 0; i < tireProfile.length; i++) {
      const nextI = (i + 1) % tireProfile.length;
      for (let j = 0; j < radialSegments; j++) {
        const nextJ = (j + 1) % radialSegments;
        const a = i * radialSegments + j;
        const b = nextI * radialSegments + j;
        const c = nextI * radialSegments + nextJ;
        const d = i * radialSegments + nextJ;
        indices.push(a, b, c, a, c, d);
      }
    }
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
    geometry.setIndex(indices);
    geometry.computeVertexNormals();
    return geometry;
  };
  const sharedTireGeometry = buildTireGeometry();

  const buildSpokesGeometry = side => {
    const positions = [];
    const addTriangle = (a, b, c) => {
      positions.push(a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2]);
    };
    const addPrism = (polygon, y0, y1) => {
      for (let i = 1; i < polygon.length - 1; i++) {
        addTriangle(
          [polygon[0][0], y1, polygon[0][1]],
          [polygon[i][0], y1, polygon[i][1]],
          [polygon[i + 1][0], y1, polygon[i + 1][1]]
        );
        addTriangle(
          [polygon[0][0], y0, polygon[0][1]],
          [polygon[i + 1][0], y0, polygon[i + 1][1]],
          [polygon[i][0], y0, polygon[i][1]]
        );
      }
      for (let i = 0; i < polygon.length; i++) {
        const next = (i + 1) % polygon.length;
        const a = [polygon[i][0], y0, polygon[i][1]];
        const b = [polygon[next][0], y0, polygon[next][1]];
        const c = [polygon[next][0], y1, polygon[next][1]];
        const d = [polygon[i][0], y1, polygon[i][1]];
        addTriangle(a, b, c);
        addTriangle(a, c, d);
      }
    };
    const point = (r, angle, tangent) => [
      r * Math.cos(angle) - tangent * Math.sin(angle),
      r * Math.sin(angle) + tangent * Math.cos(angle)
    ];
    for (let k = 0; k < 5; k++) {
      for (const pair of [-1, 1]) {
        const base = Math.PI * 2 * k / 5 + pair * 0.072 + 0.10;
        const tip = base + pair * 0.026 + side * 0.095;
        const polygon = [
          point(0.050, base, -0.015),
          point(0.276, tip, -0.028),
          point(0.276, tip, 0.028),
          point(0.050, base, 0.015)
        ];
        addPrism(polygon, side * 0.080, side * 0.104);
      }
    }
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
    geometry.computeVertexNormals();
    return geometry;
  };

  const makeWheel = (axle, side) => {
    const axleName = axle === FRONT_AXLE ? "front" : "rear";
    const sideName = side > 0 ? "left" : "right";
    const prefix = `${axleName}-${sideName}`;
    const wheel = new THREE.Group();
    wheel.name = `${prefix}-wheel`;
    wheel.position.set(axle, side * TRACK_HALF, AXLE_Z);
    wheel.userData = {
      axleDirection: "Y",
      tireRadiusMeters: TIRE_RADIUS,
      tireContactZ: 0,
      nominalArchRadialGapMeters: ARCH_RADIUS - TIRE_RADIUS
    };
    group.add(wheel);

    addMesh(wheel, `${prefix}-tire`, sharedTireGeometry, tireMaterial);

    const barrelGeometry = new THREE.CylinderGeometry(0.286, 0.286, 0.15, 64, 1, true);
    addMesh(wheel, `${prefix}-rim-barrel`, barrelGeometry, rimMaterial);

    const brakeGeometry = new THREE.CylinderGeometry(0.225, 0.225, 0.012, 48, 1, false);
    const brake = addMesh(wheel, `${prefix}-brake-disc`, brakeGeometry, brakeMaterial);
    brake.position.y = side * 0.035;

    addMesh(wheel, `${prefix}-multi-spoke-wheel-face`, buildSpokesGeometry(side), rimMaterial);

    const lipGeometry = new THREE.TorusGeometry(0.284, 0.007, 8, 64);
    lipGeometry.rotateX(Math.PI / 2);
    const lip = addMesh(wheel, `${prefix}-circular-rim-lip`, lipGeometry, rimEdgeMaterial);
    lip.position.y = side * 0.104;

    const innerRingGeometry = new THREE.TorusGeometry(0.247, 0.004, 6, 56);
    innerRingGeometry.rotateX(Math.PI / 2);
    const innerRing = addMesh(wheel, `${prefix}-inner-rim-ring`, innerRingGeometry, rimMaterial);
    innerRing.position.y = side * 0.103;

    const hubGeometry = new THREE.CylinderGeometry(0.048, 0.054, 0.036, 36, 1, false);
    const hub = addMesh(wheel, `${prefix}-hub`, hubGeometry, rimMaterial);
    hub.position.y = side * 0.094;

    const capGeometry = new THREE.CylinderGeometry(0.026, 0.026, 0.012, 32, 1, false);
    const cap = addMesh(wheel, `${prefix}-hub-cap`, capGeometry, rimEdgeMaterial);
    cap.position.y = side * 0.114;
    return wheel;
  };

  for (const axle of [FRONT_AXLE, REAR_AXLE]) {
    makeWheel(axle, -1);
    makeWheel(axle, 1);
  }

  group.updateMatrixWorld(true);
  let triangleCount = 0;
  group.traverse(object => {
    if (!object.isMesh) return;
    const geometry = object.geometry;
    triangleCount += (geometry.index ? geometry.index.count : geometry.attributes.position.count) / 3;
  });
  const bounds = new THREE.Box3().setFromObject(group);
  const size = bounds.getSize(new THREE.Vector3());
  group.userData = {
    coordinateSystem: "+X front, +Y left, +Z up; ground z=0",
    assumedComparisonDimensionsMeters: { length: 4.10, width: 1.72, height: 1.20 },
    dimensionsAreAssumptions: true,
    sourceState: "predominantly closed/stowed; unlit triangular front pockets",
    wheelbaseMeters: FRONT_AXLE - REAR_AXLE,
    wheelCenterTrackMeters: TRACK_HALF * 2,
    generatedBounds: {
      min: bounds.min.toArray(),
      max: bounds.max.toArray(),
      size: size.toArray()
    },
    triangleCount: Math.round(triangleCount),
    selfCheck: {
      reviewedByCodex: true,
      approximateVisibleRimToTireDiameterRatio: 0.291 / TIRE_RADIUS,
      trueOpenWheelArchFaces: true,
      nominalArchRadialGapMeters: ARCH_RADIUS - TIRE_RADIUS,
      tireContactPlaneZ: 0,
      boundsMinZ: bounds.min.z,
      withinTriangleBudget: triangleCount < 150000,
      nearAssumedEnvelope: Math.abs(size.x - 4.10) < 0.04 && Math.abs(size.y - 1.72) < 0.04 && Math.abs(size.z - 1.20) < 0.04
    }
  };
  return group;
}
