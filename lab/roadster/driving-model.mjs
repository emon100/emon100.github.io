import * as T from './vendor/three.module.min.js';
export function createTireGeometry(radius,width=.20){
 const h=width/2,profile=[[.69,-1],[.88,-1],[.97,-.86],[1,-.68],[1,-.48],[.974,-.43],[1,-.38],[1,-.05],[.974,0],[1,.05],[1,.38],[.974,.43],[1,.48],[1,.68],[.97,.86],[.88,1],[.69,1],[.69,-1]].map(([r,y])=>new T.Vector2(r*radius,y*h)),g=new T.LatheGeometry(profile,256),p=g.getAttribute('position');
 for(let i=0;i<p.count;i++){const x=p.getX(i),y=p.getY(i),z=p.getZ(i),r=Math.hypot(x,z);if(Math.abs(y)<h*.7&&r>radius*.94){const phase=Math.atan2(x,z)/(2*Math.PI)*36+Math.abs(y)/width*.7,groove=Math.max(0,1-Math.abs(phase-Math.round(phase))/.13)*.004,scale=(r-groove)/r;p.setXYZ(i,x*scale,y,z*scale);}}
 g.computeVertexNormals();g.computeBoundingBox();g.computeBoundingSphere();return g;
}
// The refined GLB carries four separate wheels; preserve their authored geometry.
export function rigDrivingWheels(source){
 const nodes=[];source.traverse(o=>{if(Number.isInteger(o.userData.wheelIndex))nodes.push(o)});nodes.sort((a,b)=>a.userData.wheelIndex-b.userData.wheelIndex);
 if(nodes.length!==4||nodes.some((o,i)=>o.userData.wheelIndex!==i))throw Error('Refined model must contain four indexed wheels');
 const root=new T.Group();root.add(source);root.updateMatrixWorld(true);
 const centers=nodes.map(o=>o.getWorldPosition(new T.Vector3()).toArray());
 const wheels=nodes.map((node,i)=>{const pivot=new T.Group(),spin=new T.Group();pivot.position.set(...centers[i]);root.add(pivot);pivot.add(spin);spin.attach(node);return {pivot,spin};});
 root.userData={...source.userData,wheelCenters:centers,updateVehicle(frame){wheels.forEach(({pivot,spin},i)=>{pivot.rotation.z=frame.wheelSteering?.[i]??0;spin.rotation.y=frame.wheelAngles?.[i]??0;});}};
 return root;
}
