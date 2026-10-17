const fs=require('fs'); const {Parser}=require('../src/mavlink/mavlink.js');
const f=process.argv[2]; const buf=fs.readFileSync(f);
const p=new Parser(); let i=0, n=0; const names={}; let first=null,last=null;
while(i+8<buf.length){
  const ts=Number(buf.readBigUInt64BE(i)); i+=8;
  const m=buf[i]; if(m!==0xfd&&m!==0xfe){ i++; continue; }
  const hdr=m===0xfd?10:6; const len=buf[i+1]; const sig=(m===0xfd&&(buf[i+2]&1))?13:0; const total=hdr+len+2+sig;
  const msgs=p.feed(buf.subarray(i,i+total)); i+=total;
  for(const x of msgs){ names[x.name]=(names[x.name]||0)+1; n++; if(first===null)first=ts; last=ts; }
  if(n>400000)break;
}
console.log('frames',n,'stats',JSON.stringify(p.stats),'span s',((last-first)/1e6).toFixed(0));
console.log(Object.entries(names).sort((a,b)=>b[1]-a[1]).slice(0,25).map(([k,v])=>k+':'+v).join(' '));
