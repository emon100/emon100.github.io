import {t} from './i18n.mjs';

export function createLoadingProgress(host) {
 const floating=!host;
 if(!host){host=document.createElement('div');host.className='loading-toast';document.body.append(host);}
 if(!document.querySelector('link[data-loading-style]')){const link=document.createElement('link');link.rel='stylesheet';link.href=new URL('./loading.css',import.meta.url).href;link.dataset.loadingStyle='';document.head.append(link);}
 const card=document.createElement('div');card.className='loading-card';
 const title=document.createElement('strong'),row=document.createElement('div'),percent=document.createElement('span'),label=document.createElement('span'),bar=document.createElement('progress'),detail=document.createElement('small');
 title.textContent='ROADSTER';row.className='loading-row';row.append(label,percent);bar.max=100;bar.value=0;bar.setAttribute('aria-label',t('加载进度','Loading progress'));detail.className='loading-detail';card.append(title,row,bar,detail);host.replaceChildren(card);host.setAttribute('aria-busy','true');label.setAttribute('role','status');label.setAttribute('aria-live','polite');
 let previous=0,failed=false;
 function update(fraction,message,bytes){if(failed)return;previous=Math.max(previous,Math.min(1,Math.max(0,Number.isFinite(fraction)?fraction:0)));bar.value=Math.floor(previous*100);percent.textContent=bar.value+'%';if(label.textContent!==message)label.textContent=message;detail.textContent=bytes?.loaded>0?`${(bytes.loaded/1048576).toFixed(1)}${bytes.total>0?' / '+(bytes.total/1048576).toFixed(1):''} MB`:'';}
 update(0,t('准备加载…','Preparing to load…'));
 return {update,finish(){if(failed)return;update(1,t('准备就绪','Ready'));host.setAttribute('aria-busy','false');if(floating)host.remove();else host.hidden=true;},fail(message=t('加载失败，请刷新重试。','Loading failed. Please refresh to try again.')){if(failed)return;failed=true;host.setAttribute('aria-busy','false');host.dataset.failed='true';label.textContent=message;label.setAttribute('role','alert');detail.textContent='';const retry=document.createElement('button');retry.type='button';retry.textContent=t('重新加载','Reload');retry.onclick=()=>location.reload();card.append(retry);}};
}
