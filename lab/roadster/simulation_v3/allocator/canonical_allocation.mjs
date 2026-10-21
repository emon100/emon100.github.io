// See canonical_allocation.py. This is specific to the published 12-jet layout.
export function canonicalize(wrench,limits,a){
  const w=-a[3][0],l=a[4][0],z=a[4][8];if(w<=0||l<=0)throw new Error('Unsupported nozzle geometry');
  let [Fx,Fy,Fz,tx,ty,tz]=wrench;tx+=z*Fy;ty-=z*Fx;
  const base=[0,2,4,6].map(j=>Fz/4+tx*a[3][j]/(4*w*w)+ty*a[4][j]/(4*l*l)),signs=[1,-1,-1,1];
  const aa=base.map((v,i)=>(-limits[2*i+1]-v)/signs[i]),bb=base.map((v,i)=>(limits[2*i]-v)/signs[i]);
  const left=Math.max(...aa.map((v,i)=>Math.min(v,bb[i]))),right=Math.min(...aa.map((v,i)=>Math.max(v,bb[i])));
  if(left>right+1e-5)throw new Error('LP wrench outside vertical feasible interval');
  const roots=base.map((v,i)=>-v/signs[i]).sort((x,y)=>x-y),middle=(roots[1]+roots[2])/2;
  const alpha=left>right?(left+right)/2:Math.max(left,Math.min(right,middle)),forces=Array(12).fill(0);
  base.forEach((b,i)=>{const net=b+alpha*signs[i];forces[2*i]=Math.max(net,0);forces[2*i+1]=Math.max(-net,0)});
  const horizontal=[Math.max(Fx,0),Math.max(-Fx,0),Math.max(Fy,0),Math.max(-Fy,0)],delta=tz-(w*Math.abs(Fx)-l*Math.abs(Fy));
  if(delta>=0){horizontal[0]+=delta/(2*w);horizontal[1]+=delta/(2*w)}else{horizontal[2]+=-delta/(2*l);horizontal[3]+=-delta/(2*l)}
  horizontal.forEach((v,i)=>forces[i+8]=v);
  if(Math.max(...forces.map((v,i)=>v-limits[i]))>1e-5)throw new Error('Canonical force outside nozzle bounds');
  const clipped=forces.map((v,i)=>Math.max(0,Math.min(limits[i],v)));
  if(Math.max(...a.map((row,i)=>Math.abs(row.reduce((s,v,j)=>s+v*clipped[j],0)-wrench[i])))>1e-5)throw new Error('Canonical wrench changed');
  return clipped;
}
