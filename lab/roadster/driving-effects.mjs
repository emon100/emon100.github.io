import * as T from './vendor/three.module.min.js';
import {Reflector} from './vendor/Reflector.mjs';
import {mergeGeometries} from './vendor/BufferGeometryUtils.mjs';

export function brakeLightOn(frame){
 const speed=Math.abs(frame.longitudinalSpeed??0),regen=frame.powertrain?.regenPower??0;
 return !!frame.braking||(frame.mode!=='air'&&!frame.powertrain?.hold&&speed>1&&regen/(Math.max(1,frame.mass??2140)*speed)>1.3);
}

export function createVehicleLights(skin,parent=skin){
 const shell=skin.getObjectByName('Roadster_body')??skin.getObjectByName('Roadster body');
 if(!shell?.isMesh)throw Error('Vehicle lights require the refined Roadster body');
 const brakeGlow={value:.65},material=shell.material,previous=material.onBeforeCompile;
 material.onBeforeCompile=function(shader){
  previous.call(this,shader);shader.uniforms.roadsterBrakeGlow=brakeGlow;
  shader.vertexShader=shader.vertexShader.replace('#include <common>','#include <common>\nvarying float roadsterLength;').replace('#include <begin_vertex>','#include <begin_vertex>\nroadsterLength=position.x;');
  shader.fragmentShader=shader.fragmentShader.replace('#include <common>','#include <common>\nvarying float roadsterLength;\nuniform float roadsterBrakeGlow;').replace('#include <emissivemap_fragment>','#include <emissivemap_fragment>\ntotalEmissiveRadiance *= roadsterLength > .68 ? roadsterBrakeGlow : (roadsterLength < -.5 ? 7.0 : 1.0);');
 };
 material.customProgramCacheKey=()=> 'roadster-driving-lamps-v1';material.needsUpdate=true;
 const headlights=[];
 for(const side of [-1,1]){
  const lamp=new T.SpotLight(0xeaf3ff,2400,48,.30,.65,2);lamp.name='前车灯';lamp.position.set(-.595,.069,side*.235);lamp.target.position.set(-7,-.16,side*.52);shell.add(lamp,lamp.target);headlights.push(lamp);
 }
 const rear=new T.SpotLight(0xff1326,.08,6,.85,.9,2);rear.name='刹车灯地面反光';rear.position.set(.755,.15,0);rear.target.position.set(2,-.20,0);shell.add(rear,rear.target);
 skin.updateWorldMatrix(true,true);for(const lamp of headlights){parent.attach(lamp);parent.attach(lamp.target)}parent.attach(rear);parent.attach(rear.target);
 let braking=false;
 return {update(frame){braking=brakeLightOn(frame);brakeGlow.value=braking?7:.65;rear.intensity=braking?50:.08;},stats(){return {headlights:headlights.length,headlightIntensity:headlights[0].intensity,braking,tailEmissive:brakeGlow.value,rearLightIntensity:rear.intensity}}};
}

// All patches lie on one plane, so they share a single reflected scene.
export const PUDDLES=[
 [-37,-10,2.0,1.05,.2],[-16,30,1.8,1.1,-.3],[3,-10,1.35,.8,.7],
 [-77,33,2.4,1.05,.4],[22,32,1.7,.9,-.6],[37,-37,2.3,1.1,.2],[-65,-44,1.8,.85,.6]
];
export function createPuddles({mobile=false}={}){
 const patches=PUDDLES.map(([x,y,rx,ry,angle],i)=>{
  const g=new T.CircleGeometry(1,64),p=g.attributes.position;
  for(let j=1;j<p.count;j++){const a=Math.atan2(p.getY(j),p.getX(j)),r=.95+.035*Math.sin(a*3+i)+.025*Math.cos(a*5-i);p.setXY(j,p.getX(j)*r,p.getY(j)*r);}
  g.scale(rx,ry,1);g.rotateZ(angle);g.translate(x,y,0);return g;
 });
 const geometry=mergeGeometries(patches);patches.forEach(g=>g.dispose());
 const shader={...Reflector.ReflectorShader,
  vertexShader:Reflector.ReflectorShader.vertexShader.replace('varying vec4 vUv;','varying vec4 vUv;\nvarying vec2 waterUv;\nvarying vec3 waterEye;').replace('vUv = textureMatrix','waterUv=uv; waterEye=cameraPosition-(modelMatrix*vec4(position,1.0)).xyz;\nvUv = textureMatrix'),
  fragmentShader:Reflector.ReflectorShader.fragmentShader.replace('varying vec4 vUv;','varying vec4 vUv;\nvarying vec2 waterUv;\nvarying vec3 waterEye;').replace('gl_FragColor = vec4( blendOverlay( base.rgb, color ), 1.0 );',`float edge=1.0-smoothstep(.78,1.0,length((waterUv-.5)*2.0));
   float fresnel=.10+.78*pow(1.0-abs(normalize(waterEye).z),3.0);
   gl_FragColor=vec4(mix(vec3(.012,.018,.025),base.rgb,.6+.4*fresnel),edge*(.24+.68*fresnel));`)
 };
 const object=new Reflector(geometry,{textureWidth:mobile?512:768,textureHeight:mobile?512:768,multisample:0,clipBias:.0005,shader});
 object.name='地面小滩水';object.position.z=.014;object.material.transparent=true;object.material.depthWrite=false;object.renderOrder=2;
 let passes=0;const draw=object.onBeforeRender;object.onBeforeRender=function(...args){passes++;draw.apply(this,args)};
 // shortcut: Calm, flat water only; add a fluid model if wheel splashes or wet-road grip are needed.
 return {object,stats:()=>({patches:PUDDLES.length,reflectionPasses:passes,resolution:object.getRenderTarget().width}),dispose(){object.dispose();geometry.dispose()}};
}
