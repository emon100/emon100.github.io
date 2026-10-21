import * as T from './vendor/three.module.min.js';

// Display-only mist. Forces and vehicle motion are never read back from particles.
export function createJetSmoke(nozzles,{capacity=1400}={}){
 const positions=new Float32Array(capacity*3),velocities=new Float32Array(capacity*3),radii=new Float32Array(capacity),alpha=new Float32Array(capacity),age=new Float32Array(capacity),life=new Float32Array(capacity),strength=new Float32Array(capacity),carry=nozzles.map(()=>0);
 age.fill(100);const geometry=new T.BufferGeometry();geometry.setAttribute('position',new T.BufferAttribute(positions,3));geometry.setAttribute('radius',new T.BufferAttribute(radii,1));geometry.setAttribute('opacity',new T.BufferAttribute(alpha,1));
 const material=new T.ShaderMaterial({transparent:true,depthWrite:false,uniforms:{pixelScale:{value:700}},vertexShader:`attribute float radius;attribute float opacity;uniform float pixelScale;varying float fogAlpha;void main(){vec4 p=modelViewMatrix*vec4(position,1.0);gl_Position=projectionMatrix*p;gl_PointSize=clamp(radius*2.0*pixelScale/(projectionMatrix[2][3]<-.5?max(.1,-p.z):1.0),1.0,128.0);fogAlpha=opacity;}`,fragmentShader:`varying float fogAlpha;void main(){vec2 d=gl_PointCoord-.5;float r=length(d);if(r>.5)discard;float soft=exp(-r*r*10.0)*(1.0-smoothstep(.22,.5,r));gl_FragColor=vec4(.94,.96,.98,fogAlpha*soft);}`});
 const object=new T.Points(geometry,material);object.name='白色冷气 · 方向示意';object.frustumCulled=false;
 const origin=new T.Vector3(),direction=new T.Vector3(),quat=new T.Quaternion();let next=0,lastTime=null,seed=713,enabled=true,emitted=0;
 const random=()=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed/4294967296;};
 function reset(){age.fill(100);alpha.fill(0);radii.fill(0);carry.fill(0);next=0;seed=713;emitted=0;lastTime=null;geometry.attributes.opacity.needsUpdate=true;}
 function update(f){
  if(!enabled){lastTime=f.t;return}if(lastTime===null){lastTime=f.t;return}const dt=f.t-lastTime;if(dt<0||dt>.3){reset();lastTime=f.t;return}if(dt===0)return;lastTime=f.t;
  for(let i=0;i<capacity;i++){age[i]+=dt;if(age[i]>=life[i]){alpha[i]=0;continue}const k=i*3;for(let j=0;j<3;j++)positions[k+j]+=velocities[k+j]*dt;velocities[k]*=Math.exp(-dt*1.3);velocities[k+1]*=Math.exp(-dt*1.3);velocities[k+2]+=dt*.12;if(positions[k+2]<.055){positions[k+2]=.055;const angle=Math.atan2(positions[k+1]-f.pos[1],positions[k]-f.pos[0]),spread=Math.min(5,Math.abs(velocities[k+2])*.55);velocities[k]+=Math.cos(angle)*spread;velocities[k+1]+=Math.sin(angle)*spread;velocities[k+2]=.18;}const p=age[i]/life[i];radii[i]=.065+.85*p;alpha[i]=strength[i]*.58*(1-p)**1.3;}
  quat.set(f.quat[1],f.quat[2],f.quat[3],f.quat[0]);
  nozzles.forEach((n,index)=>{const fraction=Math.max(0,Math.min(1,(f.nozzleForces?.[index]??0)/n.maxForce));if(fraction<.004)return;carry[index]+=dt*160*Math.sqrt(fraction);origin.set(...n.position).applyQuaternion(quat).add(new T.Vector3(...f.pos));direction.set(...n.direction).applyQuaternion(quat).negate();
   while(carry[index]>=1){carry[index]--;const i=next,k=i*3;next=(next+1)%capacity;const speed=6+10*Math.sqrt(fraction),birth=random()*dt;for(let j=0;j<3;j++){velocities[k+j]=(f.vel?.[j]??0)+direction.getComponent(j)*speed+(random()-.5)*1.8;positions[k+j]=origin.getComponent(j)+direction.getComponent(j)*.06+velocities[k+j]*birth;}positions[k+2]=Math.max(.055,positions[k+2]);age[i]=0;life[i]=.55+random()*.3;strength[i]=Math.sqrt(fraction);radii[i]=.065;alpha[i]=strength[i]*.58;emitted++;}
  });
  for(const key of ['position','radius','opacity'])geometry.attributes[key].needsUpdate=true;
 }
 return {object,update,reset,setEnabled(value){enabled=value;object.visible=value;reset()},setPixelScale(value){material.uniforms.pixelScale.value=value},stats(){let active=0;for(const a of alpha)if(a>0)active++;return {active,emitted,lastTime}},dispose(){geometry.dispose();material.dispose()},geometry};
}
