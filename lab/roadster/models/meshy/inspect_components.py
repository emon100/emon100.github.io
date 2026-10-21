"""Read-only GLB topology inventory. No external dependencies."""
from pathlib import Path
import struct, json, math, hashlib

HERE = Path(__file__).resolve().parent
SOURCE = HERE / 'Meshy_AI_Silver_Wedge_Concept_1016195900_generate.glb'

def read_glb(path):
    raw = path.read_bytes()
    magic, version, total = struct.unpack_from('<III', raw)
    assert magic == 0x46546c67 and version == 2 and total == len(raw)
    chunks, offset = [], 12
    while offset < len(raw):
        length, kind = struct.unpack_from('<II', raw, offset)
        chunks.append((kind, raw[offset + 8:offset + 8 + length]))
        offset += 8 + length
    document = json.loads(next(data for kind, data in chunks if kind == 0x4e4f534a))
    binary = next(data for kind, data in chunks if kind == 0x004e4942)
    return raw, document, binary, chunks

def accessor(document, binary, index):
    a = document['accessors'][index]
    view = document['bufferViews'][a['bufferView']]
    codes = {5120: 'b', 5121: 'B', 5122: 'h', 5123: 'H', 5125: 'I', 5126: 'f'}
    counts = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3, 'VEC4': 4}
    fmt = '<' + codes[a['componentType']] * counts[a['type']]
    size = struct.calcsize(fmt)
    offset = view.get('byteOffset', 0) + a.get('byteOffset', 0)
    stride = view.get('byteStride', size)
    return [struct.unpack_from(fmt, binary, offset + i * stride) for i in range(a['count'])]

def components(positions, triangles):
    parents = list(range(len(positions)))
    def find(a):
        while parents[a] != a:
            parents[a] = parents[parents[a]]
            a = parents[a]
        return a
    def union(a, b):
        a, b = find(a), find(b)
        if a != b:
            parents[b] = a
    for a, b, c in triangles:
        union(a, b); union(a, c)
    groups = {}
    for i, t in enumerate(triangles):
        groups.setdefault(find(t[0]), []).append(i)
    return sorted(groups.values(), key=len, reverse=True)

def metrics(positions, triangles, faces, ident):
    vertices = set(v for f in faces for v in triangles[f])
    lo = [min(positions[i][k] for i in vertices) for k in range(3)]
    hi = [max(positions[i][k] for i in vertices) for k in range(3)]
    area, normal_sums, center = 0., [0., 0., 0.], [0., 0., 0.]
    for f in faces:
        a, b, c = (positions[i] for i in triangles[f])
        u, v = [b[k]-a[k] for k in range(3)], [c[k]-a[k] for k in range(3)]
        n = [u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0]]
        w = math.sqrt(sum(x*x for x in n)) / 2
        area += w
        for k in range(3):
            normal_sums[k] += abs(n[k]) / 2
            center[k] += w * (a[k]+b[k]+c[k])/3
    return {'id': ident, 'triangles': len(faces), 'vertices': len(vertices), 'bounds': {'min': lo, 'max': hi}, 'span': [hi[k]-lo[k] for k in range(3)], 'area': area, 'areaWeightedCenter': [x/area for x in center] if area else center, 'absoluteNormalArea': normal_sums}

def load_source():
    raw, doc, binary, chunks = read_glb(SOURCE)
    primitive = doc['meshes'][0]['primitives'][0]
    positions = accessor(doc, binary, primitive['attributes']['POSITION'])
    indices = [i[0] for i in accessor(doc, binary, primitive['indices'])]
    triangles = [tuple(indices[i:i+3]) for i in range(0, len(indices), 3)]
    return raw, doc, binary, chunks, positions, triangles

if __name__ == '__main__':
    raw, doc, binary, chunks, positions, triangles = load_source()
    groups = components(positions, triangles)
    report = {'source': SOURCE.name, 'sha256': hashlib.sha256(raw).hexdigest(), 'sourceVertices': len(positions), 'sourceTriangles': len(triangles), 'componentMethod': 'Exact shared vertex indices; no coordinate threshold or crop', 'components': [metrics(positions, triangles, f, i) for i, f in enumerate(groups)], 'attributes': list(doc['meshes'][0]['primitives'][0]['attributes']), 'materials': len(doc.get('materials', [])), 'images': len(doc.get('images', []))}
    (HERE/'component-inspection.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
