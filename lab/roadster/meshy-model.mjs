import * as T from './vendor/three.module.min.js';
import {GLTFLoader} from './vendor/GLTFLoader.mjs';
import {assetFraction,readResponseBytes} from './asset-progress.mjs?v=20261023-loading';
import {createLoadingProgress} from './loading.mjs?v=20261023-loading';
import {MODEL_ASSET} from './asset-sizes.mjs';
import {t} from './i18n.mjs';

let loaded,lastProgress;const listeners=new Set();
export async function loadMeshyModelFactory(onProgress){
 const progress=!onProgress&&!loaded&&typeof document!=='undefined'?createLoadingProgress():null;
 const report=onProgress??(progress?event=>progress.update(assetFraction(event),event.phase==='prepare'?t('准备车辆材质…','Preparing vehicle materials…'):t('下载车辆模型…','Downloading vehicle model…'),event):null);
 if(report){listeners.add(report);if(lastProgress)report(lastProgress);}
 const emit=event=>{lastProgress=event;for(const listener of listeners)listener(event);};
 const url=new URL(MODEL_ASSET.path,import.meta.url);url.searchParams.set('revision',MODEL_ASSET.revision);
 loaded??=fetch(url).then(response=>readResponseBytes(response,emit,MODEL_ASSET.bytes)).then(bytes=>new GLTFLoader().parseAsync(bytes.buffer,new URL('.',url).href)).then(gltf=>{emit({...lastProgress,phase:'ready'});return gltf;}).catch(error=>{loaded=null;lastProgress=null;throw error;});
 let gltf;try{gltf=await loaded;progress?.finish();}catch(error){progress?.fail();throw error;}finally{if(report)listeners.delete(report);}
 return function createMeshyModel(){
  const root=new T.Group(),car=gltf.scene.clone(true),geometries=new Map(),materials=new Map(),textures=new Map();
  root.name='Meshy · 车身与底盘精修';root.userData.appearance='meshy-colored';root.userData.modelRevision='lamp-fix-20261022';
  // Source front is -X and up is +Y. Convert to +X front, +Z up without reshaping.
  car.rotation.y=Math.PI;root.rotation.x=Math.PI/2;root.add(car);
  car.traverse(o=>{if(!o.isMesh)return;
   if(!geometries.has(o.geometry))geometries.set(o.geometry,o.geometry.clone());o.geometry=geometries.get(o.geometry);
   const copyMaterial=m=>{if(!materials.has(m)){const copy=m.clone();for(const [key,value]of Object.entries(copy)){if(value?.isTexture){if(!textures.has(value)){const t=value.clone();t.needsUpdate=true;textures.set(value,t)}copy[key]=textures.get(value)}}materials.set(m,copy)}return materials.get(m)};
   o.material=Array.isArray(o.material)?o.material.map(copyMaterial):copyMaterial(o.material);o.castShadow=true;o.receiveShadow=false;
  });
  root.updateMatrixWorld(true);let box=new T.Box3().setFromObject(root),size=box.getSize(new T.Vector3());
  if(!size.toArray().every(Number.isFinite)||size.x<=0)throw Error('Meshy model has invalid bounds');
  root.scale.setScalar(4.1/size.x);root.updateMatrixWorld(true);box=new T.Box3().setFromObject(root);const center=box.getCenter(new T.Vector3());root.position.set(-center.x,-center.y,-box.min.z);
  root.userData.scaleAssumption='Uniform display length 4.1 m; dimensions uncalibrated';return root;
 };
}
