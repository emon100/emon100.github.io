// Count and broad directions follow the public Rocket prototype; dimensions and thrust remain assumptions.
export const THRUSTER_LAYOUT={id:'rocket-ten-v1',count:10,downward:4,rearward:4,sideways:2,sideExhaustElevationDegrees:5,sideAngleEvidence:'user-selected modelling assumption, not a Tesla specification'};
export function drivingNozzles(cfg){
 const a=cfg.actuators,v=cfg.vehicle,out=[],angle=THRUSTER_LAYOUT.sideExhaustElevationDegrees*Math.PI/180;
 const add=(position,direction,maxForce,role)=>out.push({id:'J'+String(out.length+1).padStart(2,'0'),position,direction,maxForce,role,evidence:'public grouping; assumed coordinates, force and side elevation'});
 for(const x of [-a.vertical_arm_x,a.vertical_arm_x])for(const y of [-a.vertical_arm_y,a.vertical_arm_y])add([x,y,a.nozzle_center_z-a.vertical_nozzle_z],[0,0,1],a.vertical_max_force,'downward_exhaust');
 for(const side of [-1,-1/3,1/3,1])add([-v.length/2-.03,side*v.track_half,a.horizontal_nozzle_z],[1,0,0],a.horizontal_max_force,'rearward_exhaust');
 for(const side of [-1,1])add([v.length/2+.03,side*(v.width/2+.025),a.horizontal_nozzle_z],[0,-side*Math.cos(angle),-Math.sin(angle)],a.horizontal_max_force,'sideways_exhaust');
 return out;
}
export function drivingNozzleXML(xml,cfg){
 const jets=drivingNozzles(cfg);
 if(!xml.includes('</body></worldbody>')||!/<actuator>[\s\S]*?<\/actuator>/.test(xml))throw Error('Unsupported driving XML');
 return xml.replace(/<site name="J\d+"[^>]*\/>/g,'').replace('</body></worldbody>',jets.map(n=>`<site name="${n.id}" pos="${n.position.join(' ')}" size=".025"/>`).join('')+'</body></worldbody>').replace(/<actuator>[\s\S]*?<\/actuator>/,`<actuator>${jets.map(n=>`<general name="${n.id}" site="${n.id}" gear="${n.direction.join(' ')} 0 0 0" ctrllimited="true" ctrlrange="0 ${n.maxForce}"/>`).join('')}</actuator>`);
}
