"""Run a real frozen recovery experiment from an explicit initial-state JSON."""
from pathlib import Path
import argparse,json
from evaluate import core,HERE,execute,verify

def main():
 p=argparse.ArgumentParser();p.add_argument('--case',help='JSON initial state; defaults to the nominal inverted 9 m case');p.add_argument('--output',required=True);p.add_argument('--baseline',action='store_true');a=p.parse_args();cfg=json.loads((HERE/'config.json').read_text());manifest=json.loads((HERE/'manifest.json').read_text());verify(manifest);case=json.loads(Path(a.case).read_text()) if a.case else core.default_case('moving_recovery',cfg)
 if case.get('task','moving_recovery')!='moving_recovery':raise ValueError('The v3 repair is only for moving_recovery')
 case['task']='moving_recovery';r=execute(case,cfg,manifest['parameters'],'baseline' if a.baseline else 'candidate',record=True);r['revision']=manifest['revision'];r['sourceSha256']=manifest['sourceSha256'];r['controllerSha256']=manifest['controllerSha256'];core.write_json(Path(a.output),r);print(json.dumps({'status':r['status'],'success':r['success'],'metrics':r['metrics'],'output':str(Path(a.output).resolve())},ensure_ascii=False))
if __name__=='__main__':main()
