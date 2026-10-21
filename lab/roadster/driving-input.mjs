export function bindDrivingControls(viewport,onChange){
 const doc=viewport.ownerDocument,win=doc.defaultView,buttons=[...viewport.querySelectorAll('[data-key]')],keys=new Set(),pointers=new Map(),listeners=[];
 const listen=(target,type,handler,options)=>{target.addEventListener(type,handler,options);listeners.push(()=>target.removeEventListener(type,handler,options));};
 const has=key=>keys.has(key)||[...pointers.values()].some(p=>p.key===key);
 function emit(){for(const b of buttons){const held=has(b.dataset.key);b.classList.toggle('held',held);b.setAttribute('aria-pressed',String(held));}onChange({throttle:Number(has('KeyW')||has('ArrowUp'))-Number(has('KeyS')||has('ArrowDown')),steer:Number(has('KeyA')||has('ArrowLeft'))-Number(has('KeyD')||has('ArrowRight')),brake:has('Space')});}
 function release(id){const p=pointers.get(id);if(!p)return;pointers.delete(id);emit();try{if(p.button.hasPointerCapture(id))p.button.releasePointerCapture(id);}catch{}}
 function clear(){const held=[...pointers];pointers.clear();keys.clear();emit();for(const [id,p] of held)try{if(p.button.hasPointerCapture(id))p.button.releasePointerCapture(id);}catch{}}
 for(const b of buttons)listen(b,'pointerdown',e=>{if(e.button!==0)return;e.preventDefault();pointers.set(e.pointerId,{key:b.dataset.key,button:b,type:e.pointerType});emit();try{b.setPointerCapture(e.pointerId);}catch{}},{passive:false});
 // Browser menus and gesture takeovers can end a press outside its original button.
 for(const type of ['pointerup','pointercancel','lostpointercapture'])listen(win,type,e=>release(e.pointerId),true);
 listen(win,'pointermove',e=>{if(e.buttons===0)release(e.pointerId);},true);
 listen(win,'mouseup',()=>{for(const [id,p] of pointers)if(p.type!=='touch')release(id);},true);
 listen(win,'touchend',e=>{if(e.touches.length===0)for(const [id,p] of pointers)if(p.type==='touch')release(id);},true);
 listen(win,'touchcancel',clear,true);
 for(const type of ['blur','focus','pagehide','orientationchange'])listen(win,type,clear);
 listen(doc,'visibilitychange',()=>{if(doc.hidden)clear();});listen(doc,'fullscreenchange',clear);
 for(const type of ['contextmenu','selectstart','dragstart'])listen(viewport,type,e=>{e.preventDefault();clear();},{capture:true,passive:false});
 listen(doc,'selectionchange',()=>{const selection=doc.getSelection();if(selection&&!selection.isCollapsed&&(viewport.contains(selection.anchorNode)||viewport.contains(selection.focusNode))){clear();selection.removeAllRanges();}});
 emit();return {has,pressKey(key){if(!keys.has(key)){keys.add(key);emit();}},releaseKey(key){if(keys.delete(key))emit();},clear,dispose(){clear();for(const remove of listeners)remove();}};
}
