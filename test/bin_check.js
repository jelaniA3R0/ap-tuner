const {parseBin}=require('../src/log/dfreader.js');
const t0=Date.now();
const log=parseBin(process.argv[2],{want:['ATT','RATE','PIDR','PIDP','PIDY','PIQR','PIQP','PIQY','IMU','RCOU','RCIN','CTUN','QTUN','MODE','AETR','VIBE','ISBH','ISBD','GYR','FTN1','FTN2','ESC','BAT']});
console.log('parsed in',Date.now()-t0,'ms; counts:',Object.entries(log.counts).filter(([k,v])=>v>1000).map(([k,v])=>k+':'+v).join(' '));
const a=log.msgs.ATT; console.log('ATT n',a.n,'t0',a.TimeUS[0]/1e6,'t1',a.TimeUS[a.n-1]/1e6,'rate Hz',(a.n/((a.TimeUS[a.n-1]-a.TimeUS[0])/1e6)).toFixed(1),'roll range',Math.min(...a.Roll).toFixed(1),Math.max(...a.Roll).toFixed(1));
const p=log.msgs.PIDR; if(p) console.log('PIDR n',p.n,'rate',(p.n/((p.TimeUS[p.n-1]-p.TimeUS[0])/1e6)).toFixed(1),'Tar/Act sample',p.Tar[5000].toFixed(2),p.Act[5000].toFixed(2),'FF',p.FF[5000].toFixed(3));
const im=log.msgs.IMU; if(im){ const {splitByInstance}=require('../src/log/dfreader.js'); const s=splitByInstance(im,'I'); for(const k of Object.keys(s)){const x=s[k]; console.log('IMU',k,'n',x.n,'rate',(x.n/((x.TimeUS[x.n-1]-x.TimeUS[0])/1e6)).toFixed(1),'GHz',x.GHz?x.GHz[0]:'-');}}
console.log('params',Object.keys(log.params).length,'changes',(log.params.__changes||[]).length,'modes',log.msgs.MODE?Array.from(log.msgs.MODE.Mode).join(','):'-');
console.log('text',log.text.slice(0,5).map(x=>x.msg).join(' | '));
