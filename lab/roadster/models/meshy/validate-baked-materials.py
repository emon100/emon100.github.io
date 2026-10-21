from pathlib import Path
from collections import Counter
import json,struct,math,hashlib,sys
from inspect_components import read_glb,accessor
base=Path(__file__).resolve().parent;path=Path(sys.argv[1]);sraw,sdoc,sbin,_=read_glb(base/'roadster-meshy-clean.glb');raw,doc,binary,_=read_glb(path)
def triangles(d,b):
 out=[]
 for p in d['meshes'][0]['primitives']:
  v=accessor(d,b,p['attributes']['POSITION']);ids=[i[0] for i in accessor(d,b,p['indices'])] if 'indices' in p else list(range(len(v)))
  assert all(all(math.isfinite(a) for a in point) for point in v)
  out.extend(struct.pack('<9f',*(v[ids[i]]+v[ids[i+1]]+v[ids[i+2]])) for i in range(0,len(ids),3))
 return Counter(out)
assert triangles(sdoc,sbin)==triangles(doc,binary)
assert doc['nodes'][0].get('matrix')==sdoc['nodes'][0].get('matrix')
for p in doc['meshes'][0]['primitives']:
 normals=accessor(doc,binary,p['attributes']['NORMAL']);uvs=accessor(doc,binary,p['attributes']['TEXCOORD_0'])
 assert all(abs(sum(x*x for x in n)-1)<2e-5 for n in normals)
 assert all(all(0<=x<=1 for x in uv) for uv in uvs)
 assert len(uvs)==len(normals)==121692
images=[]
for image in doc['images']:
 v=doc['bufferViews'][image['bufferView']];png=binary[v.get('byteOffset',0):v.get('byteOffset',0)+v['byteLength']]
 assert png[:8]==b'\x89PNG\r\n\x1a\n'
 w,h=struct.unpack('>II',png[16:24]);assert max(w,h)<=4096
 images.append({'name':image['name'],'width':w,'height':h,'bytes':len(png),'sha256':hashlib.sha256(png).hexdigest()})
assert len(images)==4
mat=doc['materials'][0];assert mat['pbrMetallicRoughness']['baseColorTexture']['index']==0 and mat['pbrMetallicRoughness']['metallicRoughnessTexture']['index']==1
assert mat['emissiveTexture']['index']==2 and mat['extensions']['KHR_materials_clearcoat']['clearcoatTexture']['index']==3
report={'sourceSha256':hashlib.sha256(sraw).hexdigest(),'outputSha256':hashlib.sha256(raw).hexdigest(),'sourceTriangles':40564,'outputTriangles':sum(triangles(doc,binary).values()),'allTriangleCoordinatesAndWindingIdentical':True,'nodeTransformIdentical':True,'normalsUnitLength':True,'uvRangeValid':True,'images':images,'surfaceUnchanged':'Source vertices are duplicated for UV seams; no triangle is subdivided, removed, moved or added.'}
path.with_suffix('.validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');print(json.dumps(report,ensure_ascii=False))
