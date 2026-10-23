import assert from 'node:assert/strict';
import {existsSync,readFileSync,readdirSync} from 'node:fs';
const root=new URL('./',import.meta.url),read=path=>readFileSync(new URL(path,root),'utf8');
assert.equal(read('CNAME').trim(),'www.emon100.com');
assert(existsSync(new URL('.nojekyll',root)));
assert(!/id="resume"|个人简历|Work Experience/.test(read('index.html')));
assert(read('index.html').includes('id="skills"'));
assert(!/2022年7月至今|Jul 2022 - Present/.test(read('resume.tex')));
assert(read('index.html').includes('href="/lab/roadster/"'));
const lab='lab/roadster/';
for(const path of readdirSync(new URL(lab,root),{recursive:true}).filter(path=>path.endsWith('.html')&&!path.startsWith('models/'))){
 const html=read(lab+path);
 assert(html.includes('个人学习与研究 · 与 Tesla 无隶属关系'),`${path}: research notice`);
 for(const [,asset] of html.matchAll(/(?:href|src)="([^"]*i18n\.(?:css|mjs))(?:\?[^"\s]*)?"/g)){
  const resolved=new URL(asset,new URL(lab+path,root));
  assert(resolved.href.startsWith(new URL(lab,root).href),`${path}: asset escapes Roadster directory`);
  assert(existsSync(resolved),`${path}: localization asset missing`);
 }
}
console.log('Homepage resume removal, skills link, domain, Roadster entry and nested localization paths passed.');
