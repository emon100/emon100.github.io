export async function readResponseBytes(response,onProgress=()=>{}) {
 if(!response.ok)throw Error(`HTTP ${response.status}: ${response.url}`);
 // Encoded Content-Length counts compressed bytes; streams expose decoded bytes.
 const size=Number(response.headers.get('content-length'));
 const total=!response.headers.get('content-encoding')&&Number.isFinite(size)&&size>0?size:0;
 let loaded=0;const chunks=[],reader=response.body?.getReader();
 onProgress({loaded,total,phase:'download'});
 if(reader){try{while(true){const {done,value}=await reader.read();if(done)break;chunks.push(value);loaded+=value.byteLength;onProgress({loaded,total,phase:'download'});}}finally{reader.releaseLock();}}
 else{const bytes=new Uint8Array(await response.arrayBuffer());chunks.push(bytes);loaded=bytes.byteLength;}
 const bytes=new Uint8Array(loaded);let offset=0;for(const chunk of chunks){bytes.set(chunk,offset);offset+=chunk.byteLength;}
 onProgress({loaded,total:loaded,phase:'prepare'});return bytes;
}

export function assetFraction({loaded=0,total=0,phase}={}) {
 if(phase==='ready')return 1;
 if(phase==='prepare')return .9;
 return total>0&&Number.isFinite(loaded)&&Number.isFinite(total)?Math.max(0,Math.min(.9,.9*loaded/total)):0;
}
