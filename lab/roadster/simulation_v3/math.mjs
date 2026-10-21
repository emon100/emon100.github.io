// Small explicit vector operations shared by the frozen Python controller port.
export const clamp=(x,a,b)=>Math.max(a,Math.min(b,x));
export const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0);
export const norm=a=>Math.hypot(...a);
export const add=(a,b)=>a.map((v,i)=>v+b[i]);
export const sub=(a,b)=>a.map((v,i)=>v-b[i]);
export const mul=(a,k)=>a.map(v=>v*k);
export const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
export const mv=(A,v)=>A.map(row=>dot(row,v));
export const tmv=(A,v)=>[0,1,2].map(j=>A.reduce((s,row,i)=>s+row[j]*v[i],0));
export const mm=(A,B)=>A.map(row=>[0,1,2].map(j=>dot(row,B.map(r=>r[j]))));
export const axisQ=(axis,a)=>[Math.cos(a/2),...mul(axis,Math.sin(a/2))];
export const wrap=x=>((x+Math.PI)%(2*Math.PI)+2*Math.PI)%(2*Math.PI)-Math.PI;
const qmul=(a,b)=>[a[0]*b[0]-dot(a.slice(1),b.slice(1)),...add(add(mul(b.slice(1),a[0]),mul(a.slice(1),b[0])),cross(a.slice(1),b.slice(1)))];
export function rotationError(q,target){let e=qmul([q[0],-q[1],-q[2],-q[3]],target);if(e[0]<0)e=mul(e,-1);const n=norm(e.slice(1));return mul(e.slice(1),2*Math.atan2(n,Math.max(e[0],0))/Math.max(n,1e-12));}
export const wrenchMatrix=nozzles=>Array.from({length:6},(_,i)=>nozzles.map(n=>[...n.direction,...cross(n.position,n.direction)][i]));
