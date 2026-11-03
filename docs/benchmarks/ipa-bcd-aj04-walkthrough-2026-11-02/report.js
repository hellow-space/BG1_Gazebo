/* Offline viewers. Coordinates and routes come only from the measured JSON. */
'use strict';
const D=window.IPA_REPORT, S=D.summary, r=S.parameters.resolution_m;
const W=S.parameters.width,H=S.parameters.height, $=id=>document.getElementById(id);
const fmt=(v,n=4)=>Number(v).toFixed(n), cname=i=>'C'+String(i).padStart(2,'0');
const boundedIndex=(value,max)=>Math.max(0,Math.min(max,Math.trunc(Number(value)||0)));
const colors=D.colors, point=p=>[p[0]/r,H-p[1]/r];
let selectedCell=38,eventIndex=6,orderIndex=0,vertexIndex=0,rawIndex=0,selectedCandidate=-1;
const imgs={};
const ready=Promise.all(['map','f','native_cells','covered','residual','f_residual'].map(name=>new Promise((resolve,reject)=>{
 const img=new Image();img.onload=()=>{imgs[name]=img;resolve();};img.onerror=()=>reject(Error('资源无法加载：'+name));img.src='assets/'+name+'.png';
})));
function polyContains(p,poly){let inside=false;for(let i=0,j=poly.length-1;i<poly.length;j=i++){
 const a=poly[i],b=poly[j];if(((a[1]>p[1])!==(b[1]>p[1]))&&(p[0]<(b[0]-a[0])*(p[1]-a[1])/(b[1]-a[1])+a[0]))inside=!inside;
}return inside;}
class MapView{
 constructor(id,draw){this.el=$(id);this.ctx=this.el.getContext('2d');this.draw=draw;this.cx=W/2;this.cy=H/2;this.scale=1;this.loaded=false;this.el.tabIndex=0;
  this.resize=()=>{const oldWidth=this.el.width;this.ratio=window.devicePixelRatio||1;this.el.width=Math.round(this.el.clientWidth*this.ratio);this.el.height=Math.round(this.el.clientHeight*this.ratio);if(this.sized)this.scale*=this.el.width/oldWidth;else{this.sized=true;this.fitWorld([15,17,58,67]);}this.render();};
  new ResizeObserver(this.resize).observe(this.el);
  this.el.addEventListener('wheel',e=>{e.preventDefault();const x=e.offsetX*this.ratio,y=e.offsetY*this.ratio;const before=this.fromScreen(x,y);this.scale*=Math.exp(-e.deltaY*.0013);this.scale=Math.min(160*this.ratio,Math.max(.1*this.ratio,this.scale));this.cx=before[0]-(x-this.el.width/2)/this.scale;this.cy=before[1]-(y-this.el.height/2)/this.scale;this.render();},{passive:false});
  let drag=null;
  this.el.addEventListener('pointerdown',e=>{drag={x:e.offsetX,y:e.offsetY,cx:this.cx,cy:this.cy,moved:false};this.el.setPointerCapture(e.pointerId);});
  this.el.addEventListener('pointermove',e=>{if(!drag)return;const dx=e.offsetX-drag.x,dy=e.offsetY-drag.y;if(Math.hypot(dx,dy)>4)drag.moved=true;this.cx=drag.cx-dx*this.ratio/this.scale;this.cy=drag.cy-dy*this.ratio/this.scale;this.render();});
  this.el.addEventListener('pointerup',e=>{if(drag&&!drag.moved&&this.onclick){const q=this.fromScreen(e.offsetX*this.ratio,e.offsetY*this.ratio);this.onclick([q[0]*r,(H-q[1])*r]);}drag=null;});
  ready.then(()=>{this.loaded=true;this.render();});
 }
 fromScreen(x,y){return [this.cx+(x-this.el.width/2)/this.scale,this.cy+(y-this.el.height/2)/this.scale];}
 fitWorld(box){const [x0,y0,x1,y1]=box;this.cx=(x0+x1)/2/r;this.cy=H-(y0+y1)/2/r;this.scale=Math.min(this.el.width/((x1-x0)/r),this.el.height/((y1-y0)/r))*.9;this.render();}
 focus(points,pad=.7,minSpan=1.5){if(!points.length)return;let x0=Infinity,y0=Infinity,x1=-Infinity,y1=-Infinity;for(const p of points){x0=Math.min(x0,p[0]);y0=Math.min(y0,p[1]);x1=Math.max(x1,p[0]);y1=Math.max(y1,p[1]);}let cx=(x0+x1)/2,cy=(y0+y1)/2,dx=Math.max(minSpan,x1-x0+2*pad),dy=Math.max(minSpan,y1-y0+2*pad);this.fitWorld([cx-dx/2,cy-dy/2,cx+dx/2,cy+dy/2]);}
 image(name,alpha=1){if(!imgs[name])return;this.ctx.globalAlpha=alpha;this.ctx.drawImage(imgs[name],0,0,W,H);this.ctx.globalAlpha=1;}
 line(points,color='#1877ba',width=1.2,dash=[]){if(points.length<2)return;const c=this.ctx;c.beginPath();let p=point(points[0]);c.moveTo(...p);for(let i=1;i<points.length;i++){p=point(points[i]);c.lineTo(...p);}c.strokeStyle=color;c.lineWidth=width*this.unit;c.setLineDash(dash.map(x=>x*this.unit));c.stroke();c.setLineDash([]);}
 dot(p,color='#d94245',radius=4){const c=this.ctx;c.beginPath();c.arc(...point(p),radius*this.unit,0,Math.PI*2);c.fillStyle=color;c.fill();c.strokeStyle='white';c.lineWidth=this.unit;c.stroke();}
 text(p,text,color='#15314d',size=11){const c=this.ctx;c.font=`600 ${size*this.unit}px sans-serif`;c.textAlign='center';c.textBaseline='middle';c.lineWidth=3*this.unit;c.strokeStyle='white';c.strokeText(text,...point(p));c.fillStyle=color;c.fillText(text,...point(p));}
 polygon(cell,color='#d94245',width=2){const p=cell.polygon;if(p.length>2)this.line([...p,p[0]],color,width);else this.dot(cell.center,color,5);}
 render(){if(!this.loaded||!this.el.width)return;const c=this.ctx;c.setTransform(1,0,0,1,0,0);c.fillStyle='#e2e8ef';c.fillRect(0,0,this.el.width,this.el.height);c.save();c.setTransform(this.scale,0,0,this.scale,this.el.width/2-this.cx*this.scale,this.el.height/2-this.cy*this.scale);this.unit=this.ratio/this.scale;c.imageSmoothingEnabled=false;this.image('map');this.draw(this);c.restore();
  const metres=Math.pow(10,Math.floor(Math.log10(100/this.scale*r*this.ratio)));const len=metres/r*this.scale;
  c.fillStyle='rgba(255,255,255,.9)';c.fillRect(12*this.ratio,this.el.height-44*this.ratio,Math.max(len+25*this.ratio,120*this.ratio),34*this.ratio);c.fillStyle='#18334c';c.fillRect(22*this.ratio,this.el.height-23*this.ratio,len,3*this.ratio);c.font=`${11*this.ratio}px sans-serif`;c.textAlign='left';c.fillText(fmt(metres,metres<1?2:0)+' m',22*this.ratio,this.el.height-29*this.ratio);
 }
}
const cellView=new MapView('cell-map',v=>{
 const c=D.cells[selectedCell];if($('show-f').checked)v.image('f',.4);if($('show-cells').checked)v.image('native_cells',.8);
 if($('show-labels').checked)for(const cc of D.cells)v.text(cc.center,cc.name,colors[cc.id],10);
 if($('show-grid').checked)for(const line of c.grid)v.line(line,'#147caf',1.5);
 if($('show-raw').checked)v.line(c.raw_path.slice(0,rawIndex+1),'#737f8c',1.6);
 if($('show-safe').checked)for(const line of c.safe_pieces)v.line(line,'#e7a026',1.6);
 if($('show-final').checked&&c.visit_order){v.line(D.path.slice(c.first_vertex,c.last_vertex+1),'#1877ba',1.2);for(let i=0;i<D.astar_metadata.length;i++)if(D.astar_metadata[i].region===c.id)v.line(D.astar[i],'#913cbb',2);}
 v.polygon(c);v.dot(c.center,'#d94245',3);v.text([c.center[0],c.center[1]+.15],c.name,'#d94245',12);
 if($('show-raw').checked&&c.raw_points)v.dot(c.raw_path[rawIndex],'#ff3388',5);
});
cellView.onclick=p=>{
 const near=D.cells.filter(c=>Math.hypot(c.center[0]-p[0],c.center[1]-p[1])/r*cellView.scale<12*cellView.ratio).sort((a,b)=>a.area_m2-b.area_m2);
 const hit=near[0]||D.cells.filter(c=>polyContains(p,c.polygon)).sort((a,b)=>a.area_m2-b.area_m2)[0];if(hit)selectCell(hit.id,false);
};
const eventView=new MapView('event-map',v=>{
 v.image('f',.45);for(let i=0;i<=eventIndex;i++)for(const line of D.events[i].cuts_world)v.line(line,i===eventIndex?'#d94245':'#913cbb',i===eventIndex?3:1.1);
 const e=D.events[eventIndex];v.line(e.row_world,'#d6a900',1,[6,4]);for(const p of e.critical_world)v.dot(p,'#ffe136',2.5);
});
const orderView=new MapView('order-map',v=>{
 const s=D.schedule[orderIndex],c=D.cells[s.region];v.image('f',.4);
 if($('order-candidates').checked)for(let i=0;i<s.candidates.length;i++){
  const q=s.candidates[i],win=q.region===s.region&&q.reverse===s.reverse;
  v.line(q.route,selectedCandidate===i?'#e7a026':win?'#129266':'#8d96a5',selectedCandidate===i?3:1.2);v.dot(q.point,win?'#129266':'#e7a026',4);v.text([q.point[0],q.point[1]+.12],cname(q.region)+(q.reverse?'反':'正'),'#152f48',10);
 }
 if(s.first_vertex>0)v.line(D.path.slice(0,s.first_vertex+1),'#6394c1',1);
 v.line(D.path.slice(s.first_vertex,s.last_vertex+1),'#1877ba',1.6);v.polygon(c,'#129266');v.dot(s.here,'#d94245',5);v.dot(D.path[s.last_vertex],'#129266',5);
});
const vertexView=new MapView('vertex-map',v=>{v.image('f',.4);v.line(D.path,'#b8c6d1',.7);v.line(D.path.slice(0,vertexIndex+1),'#1877ba',1.3);v.dot(D.path[vertexIndex],'#d94245',5);});
function selectCell(i,focus=false){selectedCell=Math.max(0,Math.min(79,Number(i)));const c=D.cells[selectedCell];$('cell-select').value=selectedCell;
 $('cell-info').innerHTML=`<b>${c.name}</b> · 原生标签 ${c.label} · 原生轮廓填充 ${c.native_area_pixels} 格 / ${fmt(c.area_m2,6)} m² · 标签图 ${c.native_label_pixels} 格<br>网格 ${c.grid_rows} 行 / ${c.grid_points} 点 → 原生 ${c.raw_points} 点 → 安全 ${c.safe_piece_count} 段。<b>${c.status}</b>${c.visit_order?`；访问 #${c.visit_order}，${c.reverse?'反向':'正向'}，当区含接近路线 ${fmt(c.final_length_m)} m`:'，不单独进入调度'}。`;
 $('cell-image').src=`assets/cells/${c.name}.png`;$('cell-image-link').href=$('cell-image').src;
 const max=Math.max(0,c.raw_points-1);$('raw-progress').max=max;$('raw-index').max=max;rawIndex=max;$('raw-progress').value=max;$('raw-index').value=max;updateRaw(false);
 if(c.corner_costs.length){$('corner-info').innerHTML='<p>原生 cell 初始中心（旋转图像素）：'+D.native_centers[selectedCell].join(', ')+'。选中角 '+c.chosen_corner+'；以下是原生 A* 像素路径长度，不是外层入口调度代价。</p><table><thead><tr><th>角</th><th>旋转图像素坐标</th><th>原生距离 / px</th><th>折算 / m</th></tr></thead><tbody>'+c.corner_costs.map((cost,j)=>`<tr class="${j===c.chosen_corner?'winning':''}"><td>${j} / ${['首行左','首行右','末行左','末行右'][j]}</td><td>${c.corners_rotated[j].join(', ')}</td><td>${cost>=1e99?'不可达 (1e100)':fmt(cost,6)}</td><td>${cost>=1e99?'—':fmt(cost*r,6)}</td></tr>`).join('')+'</tbody></table>';
 }else $('corner-info').innerHTML=`<p>局部二值化后白像素 ${c.rotated_white_pixels} 个，网格为空，提前返回；没有执行四角选择。</p>`;
 for(const tr of $('cells-table').rows)tr.classList.toggle('selected',Number(tr.dataset.cell)===selectedCell);
 if(focus)cellView.focus(c.polygon,.5,1.2);cellView.render();
}
function updateRaw(show=true){const c=D.cells[selectedCell];$('raw-progress').value=rawIndex;$('raw-index').value=rawIndex;$('raw-info').textContent=c.raw_points?`索引 ${rawIndex}/${c.raw_points-1}：(${fmt(c.raw_path[rawIndex][0],6)}, ${fmt(c.raw_path[rawIndex][1],6)}) m`:'原生没有目标点';if(show)$('show-raw').checked=true;cellView.render();}
function selectEvent(i,focus=false){eventIndex=Math.max(0,Math.min(65,Number(i)));const e=D.events[eventIndex];$('event-range').value=eventIndex+1;$('event-select').value=eventIndex;
 $('event-info').innerHTML=`<b>E${String(e.index).padStart(2,'0')} / ${e.type}</b> · 原生扫描行 y′=${e.y} · 片段 ${e.before} → ${e.after} · ${e.critical_points.length} 个黑像素检查通过 · 新增 ${e.cuts.length} 段边界。${e.cuts.length?'本事件确实改变分区边界。':'本事件没有新增边界，不能把它计作实际切割。'}`;
 if(focus){const pts=e.critical_world.length?e.critical_world:e.cuts_world.flat();if(pts.length)eventView.focus(pts,1,3);else eventView.focus(e.row_world,1,2);}eventView.render();
}
function selectOrder(i,focus=false){orderIndex=Math.max(0,Math.min(60,Number(i)));selectedCandidate=-1;const s=D.schedule[orderIndex],c=D.cells[s.region];$('order-range').value=orderIndex+1;$('order-select').value=orderIndex;
 $('order-info').innerHTML=`<b>第 ${s.step}/61 轮 · ${c.name} ${s.reverse?'反向':'正向'}</b> · 接近 ${fmt(c.approach_length_m,6)} m · 当区含接近 ${fmt(c.final_length_m)} m · 累计路线到顶点 ${s.last_vertex}。红点：当轮起点；绿点：访问完该区后的终点。`;
 $('candidate-table').innerHTML='<table><thead><tr><th>候选</th><th>方向</th><th>直线距离 / m</th><th>安全路线 / m</th><th>结果</th></tr></thead><tbody>'+s.candidates.map((q,j)=>{const win=q.region===s.region&&q.reverse===s.reverse;return `<tr data-candidate="${j}" class="${win?'winning':''}"><td>${cname(q.region)}</td><td>${q.reverse?'反向':'正向'}</td><td>${fmt(q.euclidean_m,6)}</td><td>${fmt(q.length_m,6)}</td><td>${win?'本轮选中':'候选评价，不计入整路'}</td></tr>`;}).join('')+'</tbody></table><p class="hint">点候选行可在图上用橙线高亮它；候选表顺序是几何距离预选顺序。</p>';
 $('candidate-table').querySelectorAll('[data-candidate]').forEach(tr=>tr.onclick=()=>{selectedCandidate=Number(tr.dataset.candidate);orderView.focus(s.candidates[selectedCandidate].route,.6,2);orderView.render();});
 if(focus)orderView.focus([s.here,...s.candidates.map(q=>q.point)],1,4);orderView.render();
}
const cumulative=new Float64Array(D.path.length);for(let i=1;i<D.path.length;i++)cumulative[i]=cumulative[i-1]+Math.hypot(D.path[i][0]-D.path[i-1][0],D.path[i][1]-D.path[i-1][1]);
function selectVertex(i){vertexIndex=boundedIndex(i,D.path.length-1);const p=D.path[vertexIndex];$('vertex-range').value=vertexIndex;$('vertex-index').value=vertexIndex;const st=D.schedule.find(s=>vertexIndex>s.first_vertex&&vertexIndex<=s.last_vertex);const a=D.astar_metadata.find(s=>vertexIndex>s.first_vertex&&vertexIndex<=s.last_vertex);
 $('vertex-info').innerHTML=`顶点 <b>${vertexIndex}/${D.path.length-1}</b>：(${fmt(p[0],9)}, ${fmt(p[1],9)}) m · 累计长度 ${fmt(cumulative[vertexIndex],6)} m · ${st?cname(st.region)+' / 访问 #'+st.step:'公共起点'} · ${vertexIndex===0?'起点':a?'当前连接为 A*':'当前连接为可见直连'}。`;vertexView.render();}
function fillCells(){const filter=$('cell-filter').value;const cells=D.cells.filter(c=>filter==='all'||filter==='visited'&&c.visit_order||filter==='missing'&&!c.visit_order||filter==='tiny'&&c.area_m2<=.01000001);
 $('cells-table').innerHTML=cells.map(c=>`<tr data-cell="${c.id}"><td><b>${c.name}</b></td><td>${c.label}</td><td>${fmt(c.area_m2,4)}</td><td>${c.native_area_pixels}</td><td>${c.grid_rows}</td><td>${c.raw_points}</td><td>${c.safe_piece_count}</td><td>${c.visit_order||'—'}</td><td>${c.astar_sections}</td><td>${c.status}</td></tr>`).join('');
 $('cell-count').textContent=`显示 ${cells.length} 块 / 总数 80`;$('cells-table').querySelectorAll('tr').forEach(tr=>tr.onclick=()=>selectCell(tr.dataset.cell,true));
}
$('cell-select').innerHTML=D.cells.map(c=>`<option value="${c.id}">${c.name} · ${fmt(c.area_m2,4)} m² · ${c.visit_order?'访问 #'+c.visit_order:'未单独访问'}</option>`).join('');
$('event-select').innerHTML=D.events.map((e,i)=>`<option value="${i}">E${String(e.index).padStart(2,'0')} · ${e.type} · y′=${e.y} · ${e.cuts.length} 段边界</option>`).join('');
$('order-select').innerHTML=D.schedule.map((s,i)=>`<option value="${i}">#${s.step} · ${cname(s.region)} ${s.reverse?'反向':'正向'}</option>`).join('');
$('events-table').innerHTML=D.events.map((e,i)=>`<tr data-event="${i}"><td>E${String(e.index).padStart(2,'0')}</td><td>${e.type}</td><td>${e.y}</td><td>${e.before} → ${e.after}</td><td>${e.critical_points.length}</td><td>${e.cuts.length}</td></tr>`).join('');
$('events-table').querySelectorAll('tr').forEach(tr=>tr.onclick=()=>{selectEvent(tr.dataset.event,true);$('event-map').scrollIntoView({block:'center'});});
$('schedule-table').innerHTML='<table><thead><tr><th>访问 #</th><th>区域</th><th>方向</th><th>接近 m</th><th>当区含接近 m</th><th>A* 连接</th></tr></thead><tbody>'+D.schedule.map((s,i)=>{const c=D.cells[s.region];return `<tr data-order="${i}"><td>${s.step}</td><td>${c.name}</td><td>${s.reverse?'反':'正'}</td><td>${fmt(c.approach_length_m,6)}</td><td>${fmt(c.final_length_m,6)}</td><td>${c.astar_sections}</td></tr>`;}).join('')+'</tbody></table>';
$('schedule-table').querySelectorAll('tr[data-order]').forEach(tr=>tr.onclick=()=>{selectOrder(tr.dataset.order,true);$('order-map').scrollIntoView({block:'center'});});
$('rotation-matrix').textContent='R =\n'+D.rotation.R.map(row=>row.map(v=>v.toFixed(12)).join('   ')).join('\n')+'\n\n旋转角 = '+D.rotation.angle_rad.toFixed(12)+' rad\n原生画布 = '+D.rotation.shape.join(' × ');
$('cell-select').onchange=e=>selectCell(e.target.value,true);$('cell-prev').onclick=()=>selectCell(selectedCell-1,true);$('cell-next').onclick=()=>selectCell(selectedCell+1,true);$('cell-focus').onclick=()=>cellView.focus(D.cells[selectedCell].polygon,.5,1.2);$('cell-reset').onclick=()=>cellView.fitWorld([15,17,58,67]);$('cell-filter').onchange=fillCells;
for(const id of ['show-cells','show-f','show-grid','show-raw','show-safe','show-final','show-labels'])$(id).onchange=()=>cellView.render();
for(const id of ['raw-progress','raw-index'])$(id).oninput=e=>{rawIndex=boundedIndex(e.target.value,Math.max(0,D.cells[selectedCell].raw_points-1));updateRaw();};
$('event-range').oninput=e=>selectEvent(Number(e.target.value)-1);$('event-select').onchange=e=>selectEvent(e.target.value,true);$('event-prev').onclick=()=>selectEvent(eventIndex-1,true);$('event-next').onclick=()=>selectEvent(eventIndex+1,true);$('event-focus').onclick=()=>selectEvent(eventIndex,true);$('event-reset').onclick=()=>eventView.fitWorld([15,17,58,67]);
$('order-range').oninput=e=>selectOrder(Number(e.target.value)-1);$('order-select').onchange=e=>selectOrder(e.target.value,true);$('order-prev').onclick=()=>selectOrder(orderIndex-1,true);$('order-next').onclick=()=>selectOrder(orderIndex+1,true);$('order-focus').onclick=()=>selectOrder(orderIndex,true);$('order-reset').onclick=()=>orderView.fitWorld([15,17,58,67]);$('order-candidates').onchange=()=>orderView.render();
$('vertex-range').oninput=e=>selectVertex(e.target.value);$('vertex-index').oninput=e=>selectVertex(e.target.value);$('vertex-focus').onclick=()=>vertexView.focus([D.path[vertexIndex]],.8,2);$('vertex-reset').onclick=()=>vertexView.fitWorld([15,17,58,67]);
fillCells();selectCell(38);selectEvent(6);selectOrder(0);selectVertex(0);
ready.then(()=>{$('load-status').textContent='数据已就绪：80 个分区、66 次事件、61 轮调度、79,225 个最终顶点。支持离线缩放与回放。';}).catch(error=>{$('load-status').textContent=error.message;});
