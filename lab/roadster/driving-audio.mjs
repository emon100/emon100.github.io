const clamp=(v,a,b)=>Math.max(a,Math.min(b,v));
export function electricSoundState(f,input={}){
 const speed=Math.abs(f.longitudinalSpeed??0),moving=clamp(speed/.8,0,1),power=Math.abs(input.throttle??0),regen=speed>.5&&(input.brake||power<.05)?1:0,road=f.mode==='air'?0:1;
 return {motorHz:65+Math.min(speed,40)*18+power*15,motorGain:moving*(.008+.022*power+.012*regen)*(road?.9:.25),humHz:110+speed*5,humGain:road*moving*.014*clamp(1-speed/11,0,1),reverseHz:260+9*Math.sin(f.t*5),reverseGain:road*moving*(f.gear==='R'?.025:0),roadGain:road*.022*clamp(speed/23,0,1)**1.4};
}
// Synthesized electric-drive and pedestrian-warning timbres; not a recording of a production Roadster.
export function createDrivingAudio(){
 let ctx,master,motor,harmonic,hum,reverse,road,filter,enabled=true,active=true,lastState=null;const sources=[];
 function tone(type){const source=ctx.createOscillator(),gain=ctx.createGain();source.type=type;gain.gain.value=0;source.connect(gain).connect(master);source.start();sources.push(source);return {source,gain};}
 function change(param,value,tau=.06){param.setTargetAtTime(value,ctx.currentTime,tau);}
 return {async unlock(){if(!enabled||!active)return;try{if(!ctx){const Audio=window.AudioContext||window.webkitAudioContext;if(!Audio)return;ctx=new Audio({latencyHint:'interactive'});master=ctx.createGain();master.gain.value=0;master.connect(ctx.destination);motor=tone('sine');harmonic=tone('sine');hum=tone('sine');reverse=tone('sine');const noise=ctx.createBuffer(1,ctx.sampleRate*2,ctx.sampleRate),data=noise.getChannelData(0);for(let i=0;i<data.length;i++)data[i]=Math.random()*2-1;const source=ctx.createBufferSource();source.buffer=noise;source.loop=true;filter=ctx.createBiquadFilter();filter.type='lowpass';filter.frequency.value=900;road=ctx.createGain();road.gain.value=0;source.connect(filter).connect(road).connect(master);source.start();sources.push(source);}if(ctx.state==='suspended')await ctx.resume();change(master.gain,enabled&&active?.65:0);}catch{}}
 ,setEnabled(value){enabled=value;if(ctx)change(master.gain,enabled&&active?.65:0);},setActive(value){active=value;if(ctx)change(master.gain,enabled&&active?.65:0);},
 update(f,input,view){lastState=electricSoundState(f,input);if(!ctx||ctx.state!=='running')return;const s=lastState,cabin=view==='driver'?.6:1;change(motor.source.frequency,s.motorHz);change(motor.gain.gain,s.motorGain*cabin);change(harmonic.source.frequency,s.motorHz*2.01);change(harmonic.gain.gain,s.motorGain*.23*cabin);change(hum.source.frequency,s.humHz);change(hum.gain.gain,s.humGain*cabin);change(reverse.source.frequency,s.reverseHz);change(reverse.gain.gain,s.reverseGain*cabin);change(road.gain,s.roadGain*cabin);},
 status(){return {enabled,active,context:ctx?.state??'locked',parameters:lastState};},dispose(){for(const s of sources)s.stop();ctx?.close();}
 };
}
