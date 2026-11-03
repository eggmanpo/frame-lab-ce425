import {analyzeFrame,EXAMPLE} from "./solver.js";
const $=id=>document.getElementById(id),form=$("params");
let results=[],which=0,tab="arrays",p={...EXAMPLE};
const fmt=(v,d=4)=>Number.isFinite(v)?Math.abs(v)<1e-10?"0":Number(v.toPrecision(6)).toLocaleString("en-US",{maximumFractionDigits:d}):"—";
const labels=["A","B","C","D"];
function seed(){Object.entries(EXAMPLE).forEach(([k,v])=>form.elements.namedItem(k).value=v)}
function inputs(){const r={};for(const k of Object.keys(EXAMPLE)){const e=form.elements.namedItem(k);if(!e.value.trim())throw Error(k+" is required");r[k]=Number(e.value)}return r}
function matrix(title,a,d=3){return '<h3 class="matrix-title">'+title+'</h3><div class="matrix-wrap"><table><thead><tr><th>#</th>'+a[0].map((_,j)=>'<th>'+(j+1)+'</th>').join("")+'</tr></thead><tbody>'+a.map((row,i)=>'<tr><th>'+(i+1)+'</th>'+row.map(v=>'<td>'+fmt(v,d)+'</td>').join("")+'</tr>').join("")+'</tbody></table></div>'}
function vector(title,a,d=5){return matrix(title,a.map(v=>[v]),d)}
function grid(title,heads,rows){return '<h3 class="matrix-title">'+title+'</h3><div class="matrix-wrap"><table><thead><tr>'+heads.map(v=>'<th>'+v+'</th>').join("")+'</tr></thead><tbody>'+rows.map(row=>'<tr>'+row.map(v=>'<td>'+v+'</td>').join("")+'</tr>').join("")+'</tbody></table></div>'}
function draw(r){
 const svg=$("figure"),W=860,H=350,coords=r.coords;
 const end=coords[2][0],minX=-2,maxX=end+2,minY=-1.2,maxY=p.h+1.7;
 const s=Math.min((W-50)/(maxX-minX),(H-40)/(maxY-minY));
 const X=x=>25+(x-minX)*s,Y=y=>H-20-(y-minY)*s;
 const line=(a,b,color="#dbeaf2",width=5,dash="")=>'<path d="M'+X(a[0])+','+Y(a[1])+' L'+X(b[0])+','+Y(b[1])+'" stroke="'+color+'" stroke-width="'+width+'" fill="none" '+(dash?'stroke-dasharray="'+dash+'"':'')+'/>';
 let out='<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 860 350" role="img" aria-label="Frame diagram">';
 // support markers
 for(const i of [0,2]){const [x,y]=coords[i];out+='<circle cx="'+X(x)+'" cy="'+Y(y)+'" r="9" fill="#0f2837" stroke="#dbeaf2" stroke-width="3"/>'}
 if(r.braced){const [x,y]=coords[3];out+='<path d="M'+X(x)+','+Y(y)+' l-15 20 h30z" fill="none" stroke="#eaf4f9" stroke-width="2"/>'}
 for(const e of r.elements){out+=line(coords[e.nodes[0]-1],coords[e.nodes[1]-1],e.type==="Truss"?"#77bde5":"#dfeaf2",e.type==="Truss"?3:5)}
 // UDL
 const by=Y(p.h),ax=X(0),bx=X(p.L_AB);
 if(p.w>0)for(let i=1;i<12;i++){let x=ax+(bx-ax)*i/12;out+='<path d="M'+x+','+(by-42)+'v30 m-5 -7 l5 7 l5 -7" stroke="#ff9e85" stroke-width="1.8" fill="none"/>'}
 out+='<text x="'+(ax+bx)/2+'" y="'+(by-52)+'" fill="#ffb59e" text-anchor="middle" font-size="17">w = '+fmt(p.w)+' kN/m</text>';
 if(p.P>0){out+='<path d="M'+(bx+90)+','+(by-23)+' h-73 m8 -7 l-8 7 l8 7" stroke="#ff9e85" fill="none" stroke-width="2"/><text x="'+(bx+52)+'" y="'+(by-35)+'" text-anchor="middle" fill="#ffb59e" font-size="14">P = '+fmt(p.P)+' kN</text>'}
 if($("deformation").checked){
  const maxD=Math.max(1e-12,...r.nodeDisplacements.map(d=>Math.hypot(d[0],d[1])),...r.elements.map(e=>e.type==="Frame"?Math.abs(e.localDisplacements[2]*e.L):0));
  const factor=Number($("scale").value)*.02*Math.max(end,p.h)/maxD;
  for(const e of r.elements){
   const a=coords[e.nodes[0]-1],dl=e.localDisplacements,L=e.L,pts=[];
   for(let j=0;j<=30;j++){const t=j/30,xx=L*t,u=dl[0]*(1-t)+dl[3]*t;let v=0;
    if(e.type==="Frame"){v=(1-3*t*t+2*t*t*t)*dl[1]+(t-2*t*t+t*t*t)*L*dl[2]+(3*t*t-2*t*t*t)*dl[4]+(-t*t+t*t*t)*L*dl[5]}
    pts.push([a[0]+e.c*xx+factor*(e.c*u-e.s*v),a[1]+e.s*xx+factor*(e.s*u+e.c*v)]);
   }
   out+='<polyline points="'+pts.map(v=>X(v[0])+','+Y(v[1])).join(' ')+'" fill="none" stroke="#42e2c3" stroke-width="2.7" '+(e.type==="Truss"?'stroke-dasharray="6 4"':'')+'/>';
  }
 }
 coords.forEach((point,i)=>{const x=X(point[0]),y=Y(point[1]);out+='<circle cx="'+x+'" cy="'+y+'" r="5" fill="#0a1821" stroke="#e9f4fb" stroke-width="2"/><text x="'+(x+9)+'" y="'+(y+20)+'" fill="#eaf3fa" font-size="18" font-weight="bold">'+labels[i]+'</text>'});
 svg.innerHTML=out+'</svg>';
}
function show(){
 if(!results.length)return;
 const r=results[which];
 draw(r);
 $("metrics").innerHTML=[
 ["B horizontal (m)",r.nodeDisplacements[1][0]],
 ["B vertical (m)",r.nodeDisplacements[1][1]],
 ["B rotation (rad)",r.nodeDisplacements[1][2]]
 ].map(([l,v])=>'<div class="metric"><span>'+l+'</span><strong>'+fmt(v,7)+'</strong></div>').join("");
 let output="";
 if(tab==="arrays"){
  output+=matrix("Initial flags",r.flags,0)+matrix("Destination ID",r.ID,0)+matrix("Element nodes IEN",r.IEN,0)+matrix("Location LM",r.LM,0);
 }else if(tab==="stiffness"){
  for(const e of r.elements)output+=matrix(e.name+" — Local element stiffness k (6×6)",e.kl,3)+matrix(e.name+" — Global element stiffness TᵀkT (6×6)",e.kg,3);
  output+=matrix("Assembled global K",r.KG,3);
 }else if(tab==="displacements"){
  output+=grid("Joint displacements",["Node","ux (m)","uy (m)","θ (rad)"],r.nodeDisplacements.map((d,i)=>[labels[i],...d.map(x=>fmt(x,8))]));
  output+=vector("Complete reordered displacement vector",r.DG,8);
 }else if(tab==="forces"){
  output+=grid("Support reactions (+X right, +Y up, +M CCW)",["Joint","Rx (kN)","Ry (kN)","Mz (kN·m)"],r.reactions.map((v,i)=>[labels[i],...v.map(x=>fmt(x,5))]));
  for(const e of r.elements)output+=vector(e.name+" — Local element end forces [N₁,V₁,M₁,N₂,V₂,M₂]",e.localEndForces,5);
  output+=vector("Assembled force vector FG",r.FG,5);
 }else if(tab==="checks"){
  output+=grid("Equilibrium of entire structure",["ΣFx (kN)","ΣFy (kN)","ΣMz about origin (kN·m)","max free residual"],[[fmt(r.equilibrium.Fx,9),fmt(r.equilibrium.Fy,9),fmt(r.equilibrium.M,9),fmt(r.residual,9)]]);
 }
 $("tables").innerHTML=output;
 const a=results[0],b=results[1];
 $("comparison").innerHTML=grid("Support moments, signed kN·m",["Support","Part 1a","Part 1b","Difference (1b−1a)"],[0,2].map(i=>[labels[i],fmt(a.reactions[i][2],5),fmt(b.reactions[i][2],5),fmt(b.reactions[i][2]-a.reactions[i][2],5)]));
}
function run(){try{p=inputs();results=[analyzeFrame(p,false),analyzeFrame(p,true)];$("figure").classList.remove("error");show()}catch(e){$("figure").innerHTML='<div class="error">'+String(e.message).replaceAll("<","&lt;")+'</div>';results=[];$("metrics").innerHTML="";$("tables").innerHTML="";$("comparison").innerHTML=""}}
function download(filename,content,type){const a=document.createElement("a"),url=URL.createObjectURL(new Blob([content],{type}));a.href=url;a.download=filename;a.click();setTimeout(()=>URL.revokeObjectURL(url),500)}
form.addEventListener("submit",e=>{e.preventDefault();run()});
$("reset").addEventListener("click",()=>{seed();run()});
$("case").addEventListener("change",e=>{which=Number(e.target.value);show()});
$("deformation").addEventListener("change",show);
$("scale").addEventListener("input",show);
$("tabs").addEventListener("click",e=>{const b=e.target.closest("[data-tab]");if(!b)return;tab=b.dataset.tab;document.querySelectorAll("[data-tab]").forEach(x=>x.classList.toggle("active",x===b));show()});
$("json").addEventListener("click",()=>{if(results.length)download("framelab-results.json",JSON.stringify({inputs:p,results},null,2),"application/json")});
$("csv").addEventListener("click",()=>{if(!results.length)return;const lines=["case,node,ux_m,uy_m,theta_rad,Rx_kN,Ry_kN,Mz_kNm"];results.forEach((r,i)=>r.nodeDisplacements.forEach((d,j)=>lines.push([i?"braced":"unbraced",labels[j],...d,...r.reactions[j]].join(","))));download("framelab-results.csv",lines.join("\n"),"text/csv")});
seed();run();
