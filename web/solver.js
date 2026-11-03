/* CE425 2D frame / axial truss solver. kN, m, rad. +X right, +Y up, +M CCW. */
export const EXAMPLE={h:4,L_AB:6,L_BC:5,EA_AB:2e6,EI_AB:2.5e4,EA_BC:2e6,EI_BC:2.5e4,EA_truss:8e5,w:12,P:50};
const zeros=(r,c)=>Array.from({length:r},()=>Array(c).fill(0));
const mv=(A,v)=>A.map(row=>row.reduce((s,x,i)=>s+x*v[i],0));
const transpose=A=>A[0].map((_,j)=>A.map(r=>r[j]));
const mult=(A,B)=>A.map(row=>B[0].map((_,j)=>row.reduce((s,x,k)=>s+x*B[k][j],0)));
function linearSolve(A,b){
 const n=b.length,a=A.map((r,i)=>[...r,b[i]]),scale=Math.max(1,...A.flat().map(Math.abs));
 for(let j=0;j<n;j++){
  let k=j;for(let i=j+1;i<n;i++)if(Math.abs(a[i][j])>Math.abs(a[k][j]))k=i;
  if(Math.abs(a[k][j])<1e-12*scale)throw Error("Singular stiffness matrix");
  [a[j],a[k]]=[a[k],a[j]];
  for(let i=j+1;i<n;i++){const f=a[i][j]/a[j][j];for(let m=j+1;m<=n;m++)a[i][m]-=f*a[j][m];a[i][j]=0}
 }
 const x=Array(n).fill(0);for(let i=n-1;i>=0;i--){let v=a[i][n];for(let j=i+1;j<n;j++)v-=a[i][j]*x[j];x[i]=v/a[i][i]}return x;
}
export function assembleID(flags){
 let f=0,c=0;return flags.map(row=>row.map(v=>v===0?++f:v===1?-(++c):(()=>{throw Error("Invalid ID flag")})()));
}
export function assembleLM(ID,IEN){return IEN.map(row=>row.flatMap(n=>ID[n-1]))}
function element(a,b,EA,EI,truss,q=[0,0]){
 const dx=b[0]-a[0],dy=b[1]-a[1],L=Math.hypot(dx,dy),c=dx/L,s=dy/L;
 if(!(L>0))throw Error("Zero-length member");
 const R=zeros(6,6),T=[[c,s,0],[-s,c,0],[0,0,1]];
 for(let i=0;i<3;i++)for(let j=0;j<3;j++){R[i][j]=T[i][j];R[i+3][j+3]=T[i][j]}
 let kl=zeros(6,6),fl=Array(6).fill(0);
 if(truss){const t=EA/L;kl[0][0]=kl[3][3]=t;kl[0][3]=kl[3][0]=-t}
 else{
  const a=EA/L,b=12*EI/L**3,d=6*EI/L**2,e=4*EI/L,f=2*EI/L;
  kl=[[a,0,0,-a,0,0],[0,b,d,0,-b,d],[0,d,e,0,-d,f],[-a,0,0,a,0,0],[0,-b,-d,0,b,-d],[0,d,f,0,-d,e]];
  const qx=c*q[0]+s*q[1],qy=-s*q[0]+c*q[1];
  fl=[qx*L/2,qy*L/2,qy*L**2/12,qx*L/2,qy*L/2,-qy*L**2/12];
 }
 return {L,c,s,R,kl,fl,kg:mult(mult(transpose(R),kl),R),fg:mv(transpose(R),fl)};
}
export function analyzeFrame(p,braced=false){
 for(const k of Object.keys(EXAMPLE))if(!Number.isFinite(p[k]))throw Error(k+" must be a finite number");
 for(const k of ["h","L_AB","L_BC","EA_AB","EI_AB","EA_BC","EI_BC","EA_truss"])if(p[k]<=0)throw Error(k+" must be positive");
 if(p.L_BC<=p.h)throw Error("L_BC must exceed the 4 m vertical drop");
 if(p.P<0||p.w<0)throw Error("P and w must be nonnegative magnitudes");
 const coords=[[0,p.h],[p.L_AB,p.h],[p.L_AB+Math.sqrt(p.L_BC**2-p.h**2),0]];
 const IEN=[[1,2],[2,3]],types=["Frame","Frame"],EA=[p.EA_AB,p.EA_BC],EI=[p.EI_AB,p.EI_BC];
 const flags=[[1,1,1],[0,0,0],[1,1,1]];
 if(braced){coords.push([0,0]);IEN.push([2,4]);types.push("Truss");EA.push(p.EA_truss);EI.push(0);flags.push([1,1,1])}
 const ID=assembleID(flags),LM=assembleLM(ID,IEN),neq=3,n=coords.length*3;
 const KG=zeros(n,n),FG=Array(n).fill(0),DG=Array(n).fill(0);
 const index=q=>q>0?q-1:neq-q-1,idx=ID.map(row=>row.map(index));
 FG[idx[1][0]]-=p.P;
 const elements=IEN.map(([a,b],i)=>{
  const m=element(coords[a-1],coords[b-1],EA[i],EI[i],types[i]==="Truss",i===0?[0,-p.w]:[0,0]);
  const map=LM[i].map(index);
  for(let j=0;j<6;j++){FG[map[j]]+=m.fg[j];for(let k=0;k<6;k++)KG[map[j]][map[k]]+=m.kg[j][k]}
  return {...m,map,nodes:[a,b],name:["AB","BC","BD"][i],type:types[i]};
 });
 const unknown=linearSolve(KG.slice(0,neq).map(r=>r.slice(0,neq)),FG.slice(0,neq));
 for(let i=0;i<neq;i++)DG[i]=unknown[i];
 const residual=mv(KG,DG).map((v,i)=>v-FG[i]);
 const nodeDisplacements=idx.map(row=>row.map(i=>DG[i]));
 const reactions=idx.map((row,k)=>row.map((i,j)=>flags[k][j]===1?residual[i]:0));
 elements.forEach(el=>{
  const dg=el.map.map(i=>DG[i]),dl=mv(el.R,dg);
  el.localDisplacements=dl;el.localEndForces=mv(el.kl,dl).map((v,i)=>v-el.fl[i]);
 });
 const Rx=reactions.reduce((s,r)=>s+r[0],0),Ry=reactions.reduce((s,r)=>s+r[1],0);
 const MR=reactions.reduce((s,r,i)=>s+r[2]+coords[i][0]*r[1]-coords[i][1]*r[0],0);
 return {braced,coords,IEN,types,flags,ID,LM,neq,KG,FG,DG,nodeDisplacements,reactions,elements,
  equilibrium:{Fx:Rx-p.P,Fy:Ry-p.w*p.L_AB,M:MR+p.P*p.h-p.w*p.L_AB**2/2},
  residual:Math.max(...residual.slice(0,neq).map(Math.abs)),parameters:{...p}};
}
