import {t as tr, getLocale} from './i18n.mjs';
let evaluationRevision=0,downloadRevision=0;
const GROUPS=[['old_fixed_recovery',tr("旧固定回正集","Old fixed recovery set")],['old_fixed_SO3',tr("旧固定任意姿态集","Old fixed arbitrary-attitude set")],['new_seed_recovery',tr("新 seed 回正集","New-seed recovery set")],['new_seed_SO3',tr("新 seed 任意姿态集","New-seed arbitrary-attitude set")]];
const fingerprints=['revision','sourceSha256','controllerSha256','allocatorSha256','casesSha256'];
const esc=s=>String(s??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const stable=x=>JSON.stringify(x,(_,v)=>v&&typeof v==='object'&&!Array.isArray(v)?Object.fromEntries(Object.keys(v).sort().map(k=>[k,v[k]])):v);
function checkedRows(rows,summary,label){
 if(!Array.isArray(rows)||!rows.length||!summary||!Number.isInteger(summary.trials)||summary.trials<1||!Number.isInteger(summary.successCount)||summary.successCount<0||summary.successCount>summary.trials)throw Error(label+tr("缺少完整统计",": incomplete statistics"));
 if(rows.length!==summary.trials||rows.some(r=>typeof r.success!=='boolean'||!r.case?.id||typeof r.status!=='string'))throw Error(label+tr("逐例结果不完整",": incomplete per-case outcomes"));
 if(new Set(rows.map(r=>r.case.id)).size!==rows.length||rows.filter(r=>r.success).length!==summary.successCount)throw Error(label+tr("计数不一致",": counts disagree"));
 return {success:summary.successCount,total:summary.trials,errors:rows.filter(r=>r.status==='computation_error').length};
}
export function evaluationView(results,audit){
 if(!results||typeof results.status!=='string')return {state:'error',message:tr("评估数据缺失或格式错误，未显示统计。","Evaluation data missing or malformed; statistics withheld.")};
 if(results.status!=='complete')return {state:'pending',message:tr("冻结后的评估尚未完成，当前状态：","Post-freeze evaluation incomplete; current status: ")+results.status+tr("。部分结果不作为最终统计。",". Partial results are not final statistics.")};
 try{
  if(fingerprints.some(k=>typeof results[k]!=='string'||!results[k]))throw Error(tr("冻结指纹不完整","Incomplete freeze fingerprints"));
  const rows=GROUPS.map(([id,label])=>{const g=results.groups?.[id];if(!g)throw Error(label+tr("缺失"," missing"));const baseline=checkedRows(g.baseline,g.baselineSummary,label+tr("原 r6"," original r6")),candidate=checkedRows(g.candidate,g.candidateSummary,label+tr("候选"," candidate"));
   const byId=new Map(g.baseline.map(r=>[r.case.id,r]));if(baseline.total!==candidate.total||g.candidate.some(r=>!byId.has(r.case.id)||stable(byId.get(r.case.id).case)!==stable(r.case)))throw Error(label+tr("配对初态不一致",": paired initial states differ"));
   return {id,label,baseline,candidate,fine:null};
  });
  const coarseDt=results.config?.physics?.dt;if(!Number.isFinite(coarseDt)||coarseDt<=0)throw Error(tr("主评估步长缺失","Main evaluation step size missing"));
  let auditState='pending',auditMessage=tr("细步长复算尚未完成。","Fine-step recomputation incomplete.");
  if(!audit||typeof audit.status!=='string'){auditState='error';auditMessage=tr("细步长数据尚不可用，未把缺失统计记为零。","Fine-step data unavailable; missing statistics are not counted as zero.")}
  else if(audit.status!=='complete')auditMessage=tr("细步长复算尚未完成，当前状态：","Fine-step recomputation incomplete; current status: ")+audit.status+'。';
  else {
   try{
    if(fingerprints.some(k=>audit[k]!==results[k]))throw Error(tr("细步长与主评估的源码或初态指纹不一致","Fine-step and main evaluation source or initial-state fingerprints disagree"));
    if(!Number.isFinite(audit.dt)||audit.dt<=0||audit.dt>=coarseDt)throw Error(tr("细步长字段无效","Invalid fine-step field"));
    const fineRows=rows.map(row=>{const g=audit.groups?.[row.id];if(!g)throw Error(row.label+tr("细步长数据缺失",": fine-step data missing"));const fine=checkedRows(g.episodes,g.summary,row.label+tr("细步长"," fine step")),source=results.groups[row.id].candidate,byId=new Map(g.episodes.map(r=>[r.case.id,r]));
     if(fine.total!==row.candidate.total||source.some(r=>!byId.has(r.case.id)||stable(byId.get(r.case.id).case)!==stable(r.case)))throw Error(row.label+tr("细步长初态不一致",": fine-step initial states differ"));
     let successToFailure=0,failureToSuccess=0,statusChanges=0,uncomparable=0;
     for(const r of source){const other=byId.get(r.case.id);if(r.status==='computation_error'||other.status==='computation_error'){uncomparable++;continue}if(r.success&&!other.success)successToFailure++;if(!r.success&&other.success)failureToSuccess++;if(r.status!==other.status)statusChanges++}
     return {...fine,successToFailure,failureToSuccess,statusChanges,uncomparable};
    });
    rows.forEach((r,i)=>r.fine=fineRows[i]);auditState='complete';auditMessage=tr("同一冻结候选与初态逐例复算；积分与固定姿态预测使用细步长，反馈间隔保持不变。原 r6 未做这轮细步长复算。","The same frozen candidate and initial states are recomputed per case. Integration and fixed-attitude prediction use the finer step; feedback interval is unchanged. Original r6 was not recomputed at this finer step.");
   }catch(error){auditState='error';auditMessage=error.message+tr("，未显示细步长统计。","; fine-step statistics withheld.")}
  }
  const ordered=[...GROUPS.filter(([id])=>id.startsWith('new_')),...GROUPS.filter(([id])=>id.startsWith('old_'))];
  const choices=ordered.flatMap(([id,label])=>results.groups[id].candidate.map(r=>({...r,group:id,label})));
  return {state:'complete',rows,coarseDt,fineDt:auditState==='complete'?audit.dt:null,auditState,auditMessage,examples:{success:choices.find(r=>r.success),failure:choices.find(r=>!r.success&&r.status!=='computation_error')}};
 }catch(error){return {state:'error',message:error.message+tr("，未显示最终统计。","; final statistics withheld.")}}
}
export function halfStepView(results,audit,comparison,view=evaluationView(results,audit)){
 if(view.state!=='complete'||view.auditState!=='complete')return {state:'pending',message:tr("等待主评估与细步长复算完成，暂不显示0.5 ms复核。","Waiting for main and fine-step evaluation; 0.5 ms review withheld.")};
 if(!comparison||typeof comparison.status!=='string')return {state:'pending',message:tr("0.5 ms变化案例复核尚未就绪。","0.5 ms changed-case review is not ready.")};
 if(comparison.status!=='complete')return {state:'pending',message:tr("0.5 ms变化案例复核尚未完成，当前状态：","0.5 ms changed-case review incomplete; current status: ")+comparison.status+'。'};
 try{
  if([...fingerprints,'predictorSha256'].some(k=>!results[k]||comparison[k]!==results[k]))throw Error(tr("0.5 ms复核指纹不一致","0.5 ms review fingerprints disagree"));
  if(!Array.isArray(comparison.changed)||!Number.isInteger(comparison.extraEvaluations)||comparison.extraEvaluations!==comparison.changed.length)throw Error(tr("0.5 ms复核计数缺失或不一致","0.5 ms review counts missing or inconsistent"));
  const expected=new Map();for(const row of view.rows){const other=new Map(audit.groups[row.id].episodes.map(r=>[r.case.id,r]));for(const r of results.groups[row.id].candidate){const fine=other.get(r.case.id);if(r.status!=='computation_error'&&fine.status!=='computation_error'&&r.success!==fine.success)expected.set(row.id+'/'+r.case.id,{coarse:r,fine,label:row.label})}}
  if(expected.size!==comparison.changed.length)throw Error(tr("0.5 ms复核案例数不等于成功分类变化数","0.5 ms review case count differs from success-classification change count"));
  const rows=comparison.changed.map(r=>{const key=r.group+'/'+r.case?.id,match=expected.get(key);if(!match||stable(r.case)!==stable(match.coarse.case))throw Error(tr("0.5 ms复核初态缺失或重复","0.5 ms review initial state missing or duplicated"));
   for(const [name,source]of [['dt2ms',match.coarse],['dt1ms',match.fine]])if(r[name]?.success!==source.success||r[name]?.status!==source.status)throw Error(tr("0.5 ms记录中的前序结果不匹配","Earlier outcomes in 0.5 ms record do not match"));
   if(typeof r.dtHalfms?.success!=='boolean'||typeof r.dtHalfms?.status!=='string')throw Error(tr("0.5 ms结果缺失","0.5 ms outcome missing"));expected.delete(key);return {...r,label:match.label};
  });return {state:'complete',rows,count:rows.length};
 }catch(error){return {state:'error',message:error.message+tr("，未显示0.5 ms统计。","; 0.5 ms statistics withheld.")}}
}
function halfStepMarkup(results,audit,comparison,view){
 const h=halfStepView(results,audit,comparison,view);if(h.state!=='complete')return `${tr("<div class=\"rv-half-step\"><h3>0.5 ms复核范围</h3><p class=\"","<div class=\"rv-half-step\"><h3>0.5 ms review scope</h3><p class=\"")}${h.state==='error'?'rv-bad':'small'}">${esc(h.message)}</p></div>`;
 const label=r=>r.status==='computation_error'?tr("计算错误","Computation error"):r.success?tr("通过","Pass"):r.status==='hard_contact'?tr("冲击超限","Impact limit exceeded"):r.status==='body_contact'?tr("车身触地","Body contact"):esc(r.status);
 return `${tr("<div class=\"rv-half-step\"><h3>0.5 ms只复核分类变化案例</h3><p class=\"small\">仅对2 ms与1 ms之间成功分类发生变化的 ","<div class=\"rv-half-step\"><h3>0.5 ms reviews changed classifications only</h3><p class=\"small\">Additional 0.5 ms review covers only the ")}${h.count}${tr(" 个案例追加0.5 ms复核。没有进行全量0.5 ms评估；这些案例不能用于计算整组的0.5 ms成功率。控制器与参数保持冻结。</p><div class=\"rv-table-scroll\"><table><thead><tr><th>变化案例</th><th>2 ms</th><th>1 ms</th><th>0.5 ms</th></tr></thead><tbody>"," cases whose success classification changed between 2 ms and 1 ms. There was no full 0.5 ms evaluation; these cases cannot establish a group-wide 0.5 ms success rate. Controller and parameters remain frozen.</p><div class=\"rv-table-scroll\"><table><thead><tr><th>Changed case</th><th>2 ms</th><th>1 ms</th><th>0.5 ms</th></tr></thead><tbody>")}${h.rows.map(r=>`<tr><th>${esc(r.label)}<br><span class="small rv-case-id">${esc(r.case.id)}</span></th>${['dt2ms','dt1ms','dtHalfms'].map(k=>`<td class="${r[k].success?'rv-good':'rv-bad'}">${label(r[k])}</td>`).join('')}</tr>`).join('')}${tr("</tbody></table></div><p class=\"small\">接触边界可能随步长改变，0.5 ms通过不能据此消除1 ms失败记录。<a href=\"simulation_v3/step-comparison.json\" download>下载变化案例复核</a></p></div>","</tbody></table></div><p class=\"small\">Contact boundaries can change with step size; a 0.5 ms pass does not erase a 1 ms failure. <a href=\"simulation_v3/step-comparison.json\" download>Download changed-case review</a></p></div>")}`;
}
function caseButton(row,label){return row?`<article><h3>${label}</h3><p class="small">${esc(row.label)} · ${esc(row.case.id)}${tr("<br>记录结果 ","<br>Recorded outcome ")}${esc(row.status)}</p><button data-evaluation-case="${esc(row.case.id)}${tr("\">填入自由试验</button></article>","\">Use in free trial</button></article>")}`:`<article><h3>${label}${tr("</h3><p class=\"small\">此类样例未记录，未生成替代初态。</p></article>","</h3><p class=\"small\">No example of this type was recorded; no substitute initial state was generated.</p></article>")}`}
export function renderEvaluation(root,results,audit,comparison=null){
 const v=evaluationView(results,audit);if(v.state!=='complete'){root.innerHTML=`<p class="${v.state==='error'?'rv-bad':'small'}" role="status">${esc(v.message)}</p>`;return v}
 const count=x=>`${x.success} / ${x.total}${x.errors?`${tr("<br><span class=\"rv-bad\">含 ","<br><span class=\"rv-bad\">Includes ")}${x.errors}${tr(" 条计算错误</span>"," computation errors</span>")}`:''}`;
 root.innerHTML=`${tr("<p class=\"small\">主积分 ","<p class=\"small\">Main integration ")}${(v.coarseDt*1000).toFixed(1)} ms；${v.auditState==='complete'?tr("复算 ","Recomputed ")+(v.fineDt*1000).toFixed(1)+' ms':tr("细步长统计待核对","Fine-step statistics pending verification")}${tr("。所有分母包含指定的全部案例，计算错误单列，不能当作物理失败解释。</p><div class=\"rv-table-scroll\"><table><thead><tr><th>冻结评估组</th><th>原 r6<br>成功 / 总数</th><th>新候选<br>成功 / 总数</th><th>新候选细步长<br>成功 / 总数</th><th>逐例变化</th></tr></thead><tbody>",". All denominators include every specified case; computation errors are listed separately and must not be interpreted as physical failures.</p><div class=\"rv-table-scroll\"><table><thead><tr><th>Frozen evaluation group</th><th>Original r6<br>Success / Total</th><th>New candidate<br>Success / Total</th><th>New candidate, fine step<br>Success / Total</th><th>Per-case changes</th></tr></thead><tbody>")}${v.rows.map(r=>`<tr><th>${r.label}</th><td>${count(r.baseline)}</td><td>${count(r.candidate)}</td><td>${r.fine?count(r.fine):tr("未完成 / 不可用","Incomplete / Unavailable")}</td><td>${r.fine?`${tr("成功→失败 ","Success→Failure ")}${r.fine.successToFailure}${tr("<br>失败→成功 ","<br>Failure→Success ")}${r.fine.failureToSuccess}${tr("<br><span class=\"small\">结果类别变化 ","<br><span class=\"small\">Outcome-category changes ")}${r.fine.statusChanges}${r.fine.uncomparable?tr("；计算错误未比较 ","; computation errors not compared ")+r.fine.uncomparable:''}</span>`:tr("未比较","Not compared")}</td></tr>`).join('')}</tbody></table></div><p class="${v.auditState==='error'?'rv-bad':'small'}">${esc(v.auditMessage)}</p>${halfStepMarkup(results,audit,comparison,v)}<div class="rv-evaluation-examples">${caseButton(v.examples.success,tr("候选成功样例","Candidate success example"))}${caseButton(v.examples.failure,tr("候选失败样例","Candidate failure example"))}${tr("</div><p class=\"small\">按钮只复制记录中的完整初态。自由试验会重新计算；这里的成功与失败来自原始记录，不由浏览器预先指定。</p>","</div><p class=\"small\">Buttons copy only the complete recorded initial state. Free trials recompute physics; success and failure here come from original records, not browser presets.</p>")}`;
 const examples=Object.values(v.examples).filter(Boolean);for(const button of root.querySelectorAll('[data-evaluation-case]'))button.onclick=()=>{const row=examples.find(r=>r.case.id===button.dataset.evaluationCase);if(!row)return;try{sessionStorage.setItem('roadsterRecoveryCase',JSON.stringify(row.case));location.href='r6-lab.html?recovery=v3&lang='+getLocale()+'#sandbox'}catch(error){button.insertAdjacentHTML('afterend',`${tr("<p class=\"rv-bad\">无法保存初态，未跳转：","<p class=\"rv-bad\">Cannot save initial state; navigation canceled: ")}${esc(error.message)}</p>`)}};
 return v;
}
async function readJson(path){const r=await fetch(path,{cache:'no-store'});if(!r.ok)throw Error(path+' HTTP '+r.status);return r.json()}
export async function loadEvaluation(){
 const revision=++evaluationRevision,target=document.querySelector('#frozen-evaluation-data');target.textContent=tr("读取冻结评估与步长复算记录…","Reading frozen evaluation and step-size records…");
 const [r,a,c]=await Promise.allSettled([readJson('simulation_v3/results.json'),readJson('simulation_v3/step-audit.json'),readJson('simulation_v3/step-comparison.json')]);
 if(revision!==evaluationRevision)return {superseded:true};
 if(r.status==='rejected'){target.innerHTML=tr("<p class=\"rv-bad\" role=\"status\">评估记录尚未就绪或读取失败，未显示最终统计。</p>","<p class=\"rv-bad\" role=\"status\">Evaluation records unavailable or failed to load; final statistics withheld.</p>");return null}
 return renderEvaluation(target,r.value,a.status==='fulfilled'?a.value:null,c.status==='fulfilled'?c.value:null);
}
export async function loadRecordDownloads(){
 const revision=++downloadRevision,target=document.querySelector('#evaluation-downloads');
 try{
  const index=await readJson('simulation_v3/record-index.json');if(!Array.isArray(index.archives)||!index.archives.length)throw Error(tr("无归档","No archive"));
  const archives=index.archives.map(a=>{if(typeof a.path!=='string'||!a.path.endsWith('.zip')||a.path.split('/').some(x=>x==='..')||/^[/.]|:|\\/.test(a.path)||!Number.isFinite(a.bytes)||a.bytes<=0)throw Error(tr("归档索引字段无效","Invalid archive index fields"));return a});
  const available=await Promise.all(archives.map(async a=>{try{const r=await fetch('simulation_v3/'+a.path,{method:'HEAD',cache:'no-store'});return r.ok?a:null}catch{return null}}));
  if(revision!==downloadRevision)return;
  const present=available.filter(Boolean);target.innerHTML=`${tr("<p><a href=\"simulation_v3/record-index.json\" download>下载原始记录索引</a></p>","<p><a href=\"simulation_v3/record-index.json\" download>Download original record index</a></p>")}${present.length?`<ul>${present.map(a=>`<li><a href="simulation_v3/${esc(a.path)}" download>${esc(a.path)}</a> · ${(a.bytes/1024/1024).toFixed(2)} MiB${a.group?' · '+esc(a.group):''}</li>`).join('')}</ul>`:''}${present.length!==archives.length?tr("<p class=\"small\">部分或全部 ZIP 下载尚未就绪。</p>","<p class=\"small\">Some or all ZIP downloads are not ready.</p>"):''}${tr("<p class=\"small\">ZIP 保存原始逐例记录。页面只读取统计与索引，没有加载这些完整回放。</p>","<p class=\"small\">ZIPs retain original per-case records. This page reads only statistics and indexes, without loading full replays.</p>")}`;
 }catch{if(revision!==downloadRevision)return;target.innerHTML=tr("<p class=\"small\">原始记录索引与 ZIP 下载尚未就绪。</p>","<p class=\"small\">Original record index and ZIP downloads are not ready.</p>")}
}
if(typeof document!=='undefined'){loadEvaluation();loadRecordDownloads();document.querySelector('#refresh-evaluation').onclick=()=>{loadEvaluation();loadRecordDownloads()}}
