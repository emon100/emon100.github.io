export async function readResponseBytes(response,onProgress=()=>{},expectedTotal=0) {
 if(!response.ok)throw Error(`HTTP ${response.status}: ${response.url}`);
 // Encoded Content-Length counts compressed bytes; streams expose decoded bytes.
 const size=Number(response.headers.get('content-length'));
 const encoded=!!response.headers.get('content-encoding'),transferTotal=Number.isFinite(size)&&size>0?size:0;
 let total=Number.isFinite(expectedTotal)&&expectedTotal>0?expectedTotal:!encoded?transferTotal:0;
 let loaded=0;const chunks=[],reader=response.body?.getReader();
 const report=phase=>onProgress({loaded,total,encoded,transferTotal,phase});
 report('download');
 if(reader){try{while(true){const {done,value}=await reader.read();if(done)break;chunks.push(value);loaded+=value.byteLength;if(total>0)total=Math.max(total,loaded);report('download');}}finally{reader.releaseLock();}}
 else{const bytes=new Uint8Array(await response.arrayBuffer());chunks.push(bytes);loaded=bytes.byteLength;}
 const bytes=new Uint8Array(loaded);let offset=0;for(const chunk of chunks){bytes.set(chunk,offset);offset+=chunk.byteLength;}
 total=loaded;report('prepare');return bytes;
}

export function assetFraction({loaded=0,total=0,phase}={}) {
 if(phase==='ready')return 1;
 if(phase==='prepare')return .9;
 return total>0&&Number.isFinite(loaded)&&Number.isFinite(total)?Math.max(0,Math.min(.9,.9*loaded/total)):0;
}
