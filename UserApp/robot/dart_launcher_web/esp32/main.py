# main.py — 制导飞镖发射架 · WiFi 六界面控制台 (ESP32-S3 / MicroPython)
# 版本: dart_launcher_web v0.2
#
# 手机网页 --WiFi--> ESP32 --UART--> C板 --CAN--> 四电机
# 网页 -> ESP32 (/cmd?c=...)  直接转发给 C 板
# ESP32 -> 网页 (/state)      C 板遥测帧 F,...
#
# 六个界面: 1 识别连接 | 2 拉簧A/B | 3 扳机丝杆 | 4 Yaw | 5 总控 | 6 比赛自动化

import network
import socket
import time
from machine import UART

# ======================== 配置区 ========================
AP_SSID = "DART_CTRL"
AP_PASS = "12345678"
UART_ID = 1
UART_TX = 17
UART_RX = 18
BAUD = 115200
# =======================================================

PAGE = r"""<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<meta name="theme-color" content="#0b0f14">
<title>飞镖架控制台</title>
<style>
:root{
  --bg:#0b0f14; --surf:#151b23; --surf2:#1d2530; --line:#2a3542;
  --txt:#e8eef6; --dim:#8a97a8; --acc:#3b82f6; --acc2:#22d3ee;
  --good:#22c55e; --warn:#f59e0b; --bad:#ef4444;
  --r:14px;
}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
html,body{margin:0;padding:0}
body{
  background:var(--bg);color:var(--txt);
  font-family:-apple-system,BlinkMacSystemFont,"PingFang SC","Microsoft YaHei",Roboto,sans-serif;
  font-size:14px;padding-bottom:74px;
}
.topbar{
  position:sticky;top:0;z-index:20;display:flex;align-items:center;justify-content:space-between;
  padding:12px 14px;background:rgba(11,15,20,.92);backdrop-filter:blur(8px);
  border-bottom:1px solid var(--line);
}
.brand{font-size:16px;font-weight:700;letter-spacing:.5px}
.brand small{color:var(--dim);font-weight:400;margin-left:6px}
.pill{display:flex;align-items:center;gap:6px;padding:5px 10px;border-radius:999px;
  background:var(--surf2);border:1px solid var(--line);font-size:12px;color:var(--dim)}
.pill .dot{width:8px;height:8px;border-radius:50%;background:var(--bad);transition:.2s}
.pill.ok .dot{background:var(--good);box-shadow:0 0 8px var(--good)}
main{padding:12px 12px 8px}
.page{display:none;animation:fade .18s ease}
.page.on{display:block}
@keyframes fade{from{opacity:0;transform:translateY(4px)}to{opacity:1;transform:none}}
.card{background:var(--surf);border:1px solid var(--line);border-radius:var(--r);
  padding:14px;margin-bottom:12px}
.card>h3{margin:0 0 12px;font-size:14px;font-weight:600;color:var(--txt);
  display:flex;align-items:center;gap:8px}
.card>h3::before{content:"";width:3px;height:14px;border-radius:2px;background:var(--acc)}
.card>h3 .tag{margin-left:auto;font-size:11px;color:var(--dim);font-weight:400}
.cols{display:grid;grid-template-columns:1fr;gap:12px}
@media(min-width:680px){.cols{grid-template-columns:1fr 1fr}}
.row{display:flex;align-items:center;gap:10px;margin:9px 0}
.row>label{flex:1;color:var(--dim);font-size:13px}
.row .v{font-variant-numeric:tabular-nums;color:var(--acc2);font-weight:600}
input,select{
  background:#0d131a;border:1px solid var(--line);color:var(--txt);border-radius:9px;
  padding:9px 10px;font-size:15px;outline:none;width:100%
}
input:focus{border-color:var(--acc)}
input[type=number]{width:96px;text-align:right;flex:0 0 auto}
input[type=range]{
  -webkit-appearance:none;appearance:none;background:transparent;padding:0;border:none;
  width:100%;height:44px
}
input[type=range]::-webkit-slider-runnable-track{height:6px;border-radius:3px;background:var(--surf2)}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:26px;height:26px;margin-top:-10px;
  border-radius:50%;background:var(--acc);border:3px solid #0b0f14;box-shadow:0 0 0 1px var(--acc)}
button{
  border:none;border-radius:10px;background:var(--acc);color:#fff;padding:11px 12px;
  font-size:14px;font-weight:600;cursor:pointer;transition:.12s;font-family:inherit
}
button:active{transform:scale(.97)}
button.ghost{background:var(--surf2);color:var(--txt);border:1px solid var(--line)}
button.good{background:var(--good)}button.bad{background:var(--bad)}button.warn{background:var(--warn);color:#111}
button:disabled{opacity:.4}
.btns{display:grid;grid-template-columns:repeat(2,1fr);gap:8px;margin-top:10px}
.btns.c3{grid-template-columns:repeat(3,1fr)}
.btns.c4{grid-template-columns:repeat(4,1fr)}
.big{font-size:30px;font-weight:800;font-variant-numeric:tabular-nums;text-align:center;
  letter-spacing:.5px;margin:2px 0 6px}
.big small{font-size:13px;color:var(--dim);font-weight:400;margin-left:4px}
.sub{color:var(--dim);font-size:12px;text-align:center;margin-bottom:8px}
.mcard{background:var(--surf);border:1px solid var(--line);border-radius:var(--r);padding:14px;margin-bottom:12px}
.mhead{display:flex;align-items:center;gap:8px;margin-bottom:8px}
.mhead .name{font-weight:700;font-size:15px}
.mhead .id{font-size:11px;color:var(--dim);background:var(--surf2);border:1px solid var(--line);
  padding:2px 7px;border-radius:999px}
.led{width:9px;height:9px;border-radius:50%;background:var(--bad);margin-left:auto}
.led.on{background:var(--good);box-shadow:0 0 8px var(--good)}
.mgrid{display:grid;grid-template-columns:repeat(4,1fr);gap:8px;margin:10px 0}
.mgrid .cell{background:#0d131a;border:1px solid var(--line);border-radius:10px;padding:8px 6px;text-align:center}
.mgrid .cell b{display:block;font-size:16px;font-variant-numeric:tabular-nums;color:var(--acc2)}
.mgrid .cell span{font-size:11px;color:var(--dim)}
.seg{display:flex;gap:6px;background:#0d131a;border:1px solid var(--line);border-radius:11px;padding:4px}
.seg button{flex:1;background:transparent;color:var(--dim);padding:9px 4px;font-size:13px;font-weight:500}
.seg button.on{background:var(--acc);color:#fff}
details.pid{background:var(--surf);border:1px solid var(--line);border-radius:var(--r);margin-bottom:12px;overflow:hidden}
details.pid>summary{list-style:none;cursor:pointer;padding:14px;font-weight:600;display:flex;align-items:center;gap:8px}
details.pid>summary::-webkit-details-marker{display:none}
details.pid>summary::before{content:"";width:3px;height:14px;border-radius:2px;background:var(--acc2)}
details.pid>summary .tag{margin-left:auto;color:var(--dim);font-size:11px;font-weight:400}
details.pid .body{padding:0 14px 14px}
table{width:100%;border-collapse:collapse}
th,td{padding:8px 4px;border-bottom:1px solid var(--line);text-align:center;font-size:13px}
th{color:var(--dim);font-weight:500}
.timeline{display:flex;flex-direction:column;gap:2px}
.step{display:flex;align-items:center;gap:10px;padding:9px 10px;border-radius:10px;color:var(--dim)}
.step .n{width:22px;height:22px;border-radius:50%;background:var(--surf2);border:1px solid var(--line);
  display:flex;align-items:center;justify-content:center;font-size:11px;flex:0 0 auto}
.step.on{background:rgba(59,130,246,.12);color:var(--txt)}
.step.on .n{background:var(--acc);border-color:var(--acc);color:#fff}
.step.done{color:var(--good)}
.step.done .n{background:rgba(34,197,94,.18);border-color:var(--good);color:var(--good)}
.dgrid{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}
@media(min-width:520px){.dgrid{grid-template-columns:repeat(4,1fr)}}
.dbtn{background:var(--surf2);border:1px solid var(--line);border-radius:12px;padding:12px 6px;
  color:var(--txt);font-weight:700;font-size:16px;position:relative}
.dbtn small{display:block;font-size:11px;color:var(--dim);font-weight:400;margin-top:2px}
.dbtn .x{position:absolute;top:-6px;right:-6px;width:20px;height:20px;border-radius:50%;
  background:var(--bad);color:#fff;font-size:12px;line-height:20px;text-align:center;display:none}
.dgrid.edit .dbtn .x{display:block}
.vision{display:flex;align-items:center;gap:12px;margin:8px 0}
.vbar{position:relative;flex:1;height:10px;border-radius:5px;background:var(--surf2);overflow:hidden}
.vbar .tgt{position:absolute;left:50%;top:-3px;width:2px;height:16px;background:var(--dim)}
.vbar .mark{position:absolute;top:-2px;width:4px;height:14px;border-radius:2px;background:var(--acc2);transition:left .15s}
.estop{position:fixed;right:14px;bottom:88px;z-index:30;width:56px;height:56px;border-radius:50%;
  background:var(--bad);color:#fff;font-size:12px;font-weight:700;box-shadow:0 6px 18px rgba(239,68,68,.45)}
.tabbar{position:fixed;left:0;right:0;bottom:0;z-index:40;display:grid;grid-template-columns:repeat(6,1fr);
  background:rgba(15,20,26,.96);backdrop-filter:blur(10px);border-top:1px solid var(--line);
  padding:6px 4px calc(6px + env(safe-area-inset-bottom))}
.tabbar button{background:transparent;color:var(--dim);font-size:11px;font-weight:500;padding:8px 0;
  border-radius:10px;display:flex;flex-direction:column;align-items:center;gap:3px}
.tabbar button i{width:18px;height:18px;display:block}
.tabbar button.on{color:var(--acc)}
.tabbar button.on i{filter:drop-shadow(0 0 6px rgba(59,130,246,.7))}
.hint{color:var(--dim);font-size:12px;line-height:1.6}
.toast{position:fixed;left:50%;bottom:96px;transform:translateX(-50%) translateY(20px);
  background:#e8eef6;color:#0b0f14;padding:9px 16px;border-radius:999px;font-weight:600;font-size:13px;
  opacity:0;pointer-events:none;transition:.22s;z-index:60}
.toast.show{opacity:1;transform:translateX(-50%) translateY(0)}
</style>
</head>
<body>
<div class="topbar">
  <div class="brand">制导飞镖发射架<small>v0.2</small></div>
  <div class="pill" id="pill"><span class="dot"></span><span id="pillTxt">未连接</span></div>
</div>

<main>
  <!-- ============ 1 识别连接 ============ -->
  <section class="page on" id="pg0">
    <div class="card">
      <h3>控制总线 <span class="tag" id="busTag">CAN1 · 1Mbps</span></h3>
      <div class="row"><label>在线电机</label><span class="v" id="onlineCnt">0 / 4</span></div>
      <div class="row"><label>C 板链路</label><span class="v" id="linkTxt">--</span></div>
      <div class="btns c3">
        <button onclick="doScan()">扫描总线</button>
        <button class="ghost" onclick="cmd('Z');toast('已全部设零')">全部设零</button>
        <button class="ghost bad" onclick="cmd('G,12');toast('急停')">急停</button>
      </div>
      <div class="hint" id="scanRes" style="margin-top:10px">点击“扫描总线”识别 CAN 上的电机反馈 ID。</div>
    </div>
    <div id="cards"></div>
  </section>

  <!-- ============ 2 拉簧 A/B ============ -->
  <section class="page" id="pg1">
    <div class="hint" style="margin-bottom:10px">两个 M3508 同步驱动拉簧。标准位 = 0 圈，预备位 = 设定圈数。</div>
    <div class="cols">
      <div id="spring0"></div>
      <div id="spring1"></div>
    </div>
    <div class="cols" id="springPid0"></div>
    <div class="cols" id="springPid1"></div>
  </section>

  <!-- ============ 3 扳机丝杆 ============ -->
  <section class="page" id="pg2">
    <div id="trigCard"></div>
    <div id="trigPid"></div>
  </section>

  <!-- ============ 4 Yaw ============ -->
  <section class="page" id="pg3">
    <div class="card">
      <h3>Yaw 模式 <span class="tag">自瞄 / 制导 / 手动</span></h3>
      <div class="seg" id="yawSeg">
        <button data-m="0" onclick="setYaw(0)">手动</button>
        <button data-m="1" onclick="setYaw(1)">自瞄位</button>
        <button data-m="2" onclick="setYaw(2)">制导位 0°</button>
      </div>
      <div class="hint" style="margin-top:8px">自瞄位：相机识别绿光中心，PID 实时修正 yaw 对准；制导位：回到 0° 交给飞镖自导。</div>
    </div>
    <div class="card">
      <h3>视觉绿光 <span class="tag" id="visTag">--</span></h3>
      <div class="vision">
        <div class="vbar"><div class="tgt"></div><div class="mark" id="visMark" style="left:50%"></div></div>
        <span class="v" id="visErr" style="min-width:56px;text-align:right">--</span>
      </div>
      <div class="row"><label>当前 x / 中心</label><span class="v" id="visX">-- / --</span></div>
      <div class="row"><label>注入 x（联调）</label><input type="number" id="injX" value="160">
        <input type="number" id="injC" value="160" style="width:80px"></div>
      <div class="btns c2">
        <button class="ghost" onclick="cmd('C,'+injX.value+','+injC.value);toast('已注入视觉坐标')">注入坐标</button>
        <button class="ghost" onclick="cmd('C,160,160');toast('已回中')">回中</button>
      </div>
    </div>
    <div id="yawCard"></div>
    <div id="yawPid"></div>
  </section>

  <!-- ============ 5 总控 ============ -->
  <section class="page" id="pg4">
    <div class="card">
      <h3>四电机总控 <span class="tag">正式驱动</span></h3>
      <div class="btns c2">
        <button class="bad" onclick="stopAll()">全部停止</button>
        <button class="good" onclick="runAllAngle()">按各电机角度运行</button>
      </div>
    </div>
    <div id="master"></div>
    <div class="card">
      <h3>执行器 <span class="tag">拉簧 / 舵机</span></h3>
      <div class="row"><label>拉簧预备圈数</label><input type="number" id="mturns" step="0.1" value="5">
        <button class="ghost" onclick="cmd('W,'+Math.round(mturns.value*100));toast('圈数已保存')">保存</button></div>
      <div class="btns c4">
        <button class="ghost" onclick="cmd('G,0')">拉簧标准</button>
        <button onclick="cmd('G,1')">拉簧预备</button>
        <button class="ghost" onclick="cmd('V,2')">舵机标准</button>
        <button onclick="cmd('V,3')">舵机预备</button>
      </div>
    </div>
  </section>

  <!-- ============ 6 比赛自动化 ============ -->
  <section class="page" id="pg5">
    <div class="card">
      <h3>自动发射时序 <span class="tag" id="autoTag">空闲</span></h3>
      <div class="timeline" id="timeline"></div>
      <div class="btns c2" style="margin-top:12px">
        <button class="good" onclick="cmd('G,10');toast('开始自动发射')">开始发射</button>
        <button class="bad" onclick="cmd('G,11');toast('已停止')">停止</button>
      </div>
      <div class="hint" style="margin-top:10px">yaw 按所选模式实时修正至绿光中心；扳机电机与拉簧圈数由人工设定。</div>
    </div>
    <div class="card">
      <h3>发射参数</h3>
      <div class="row"><label>拉簧预备圈数</label><input type="number" id="aturns" step="0.1" value="5">
        <button class="ghost" onclick="cmd('W,'+Math.round(aturns.value*100));toast('圈数已保存')">保存</button></div>
      <div class="row"><label>舵机标准位 (us)</label><input type="number" id="astd" step="10" value="1000">
        <button class="ghost" onclick="cmd('V,0,'+astd.value);toast('已保存')">保存</button></div>
      <div class="row"><label>舵机预备位 (us)</label><input type="number" id="aprep" step="10" value="2000">
        <button class="ghost" onclick="cmd('V,1,'+aprep.value);toast('已保存')">保存</button></div>
      <div class="btns c2">
        <button class="ghost" onclick="cmd('V,2');toast('舵机→标准位')">舵机标准位</button>
        <button class="ghost" onclick="cmd('V,3');toast('舵机→预备位')">舵机预备位</button>
      </div>
    </div>
    <div class="card">
      <h3>距离打击表 <span class="tag" id="distTag">点击即发射</span></h3>
      <div class="dgrid" id="dgrid"></div>
      <div class="row" style="margin-top:12px">
        <input type="number" id="nd" placeholder="距离 m" step="0.1">
        <input type="number" id="nt" placeholder="圈数" step="0.1">
      </div>
      <div class="btns c2">
        <button class="ghost" onclick="addDist()">添加靶点</button>
        <button class="ghost" id="editBtn" onclick="toggleEdit()">编辑</button>
      </div>
    </div>
  </section>
</main>

<button class="estop" onclick="cmd('G,12');toast('急停！')">急停</button>
<nav class="tabbar" id="tabbar"></nav>
<div class="toast" id="toast"></div>

<script>
const MOT=[
  {n:"拉簧 A",s:"M3508 · ID2"},{n:"拉簧 B",s:"M3508 · ID3"},
  {n:"扳机丝杆",s:"M3508 · ID4"},{n:"Yaw 轴",s:"M2006 · ID1"}
];
const TF=[
  [1,"速度前馈 FF",100,0.1],[2,"速度 Kp",100,0.1],[3,"速度 Ki",100,0.1],[4,"积分限幅",100,1],
  [5,"角度 Kp",100,0.1],[7,"角度 Kd",100,0.1],[8,"角度限速",100,1],[10,"减速比",100,0.01],
  [11,"角度死区(deg)",100,0.1],[12,"角度限速(rpm)",100,1],[6,"电流/电压限幅",1,100]
];
const AUTO=["空闲","舵机 → 标准位","拉簧 → 标准位","拉簧 → 预备位 (蓄力)","舵机 → 预备位 (扣住)","拉簧 → 复位","舵机 → 复位 (发射)","完成"];
const TABS=["识别","拉簧","扳机","Yaw","总控","自动化"];
const ICONS=[
  '<circle cx="12" cy="12" r="3"/><path d="M12 2v3M12 19v3M2 12h3M19 12h3"/>',
  '<path d="M4 12h16M4 12l3-3M4 12l3 3M20 12l-3-3M20 12l-3 3"/>',
  '<circle cx="12" cy="12" r="8"/><path d="M12 8v8M8 12h8"/>',
  '<path d="M12 3v18M5 8l7-5 7 5M5 16l7 5 7-5"/>',
  '<rect x="3" y="3" width="7" height="7" rx="1"/><rect x="14" y="3" width="7" height="7" rx="1"/><rect x="3" y="14" width="7" height="7" rx="1"/><rect x="14" y="14" width="7" height="7" rx="1"/>',
  '<path d="M12 2l3 6 6 .9-4.5 4.4 1 6.2-5.5-3-5.5 3 1-6.2L3 8.9 9 8z"/>'
];
let S={n:0,motors:[],params:[],sel:-1,servo:{},springTurns:5,autoStep:0,yawMode:0,estop:0,vis:{}};
let lastRx=0, editMode=false, built=false;
const $=id=>document.getElementById(id);
function toast(t){const e=$("toast");e.textContent=t;e.classList.add("show");clearTimeout(toast._t);toast._t=setTimeout(()=>e.classList.remove("show"),1300);}
function cmd(c){fetch("/cmd?c="+encodeURIComponent(c)).catch(()=>{});}
function hb(){fetch("/ping").catch(()=>{});}

/* ---------- 标签栏 ---------- */
function buildTabs(){
  $("tabbar").innerHTML=TABS.map((t,i)=>
    '<button data-i="'+i+'" onclick="showTab('+i+')"><i><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round" width="18" height="18">'+ICONS[i]+'</svg></i>'+t+'</button>').join("");
  showTab(0);
}
function showTab(i){
  for(let k=0;k<6;k++){
    $("pg"+k).className="page"+(k===i?" on":"");
    document.querySelectorAll(".tabbar button")[k].className=(k===i?"on":"");
  }
  window.scrollTo(0,0);
}

/* ---------- 静态构建 ---------- */
function motorLiveHTML(s){return '<div class="mgrid">'+
  '<div class="cell"><b id="rpm'+s+'">0</b><span>RPM</span></div>'+
  '<div class="cell"><b id="pos'+s+'">0.0</b><span>圈</span></div>'+
  '<div class="cell"><b id="tmp'+s+'">0</b><span>°C</span></div>'+
  '<div class="cell"><b id="cur'+s+'">0.0</b><span>A</span></div></div>';}
function ctrlHTML(s,unit){
  return '<input type="range" id="sl'+s+'" min="-300" max="300" value="0" oninput="slider('+s+')">'+
  '<div class="row"><label>目标'+(unit||'角度(deg)')+'</label>'+
  '<input type="number" id="ang'+s+'" step="'+(unit?'0.1':'1')+'" value="0">'+
  '<button class="ghost" onclick="goAng('+s+')">转到</button></div>'+
  '<div class="btns c3">'+
  '<button class="ghost" onclick="cmd(\'M,'+s+',0,0\')">停止</button>'+
  '<button class="ghost" onclick="cmd(\'Z,'+s+');toast(\'已设零\')">设零</button>'+
  '<button class="ghost" onclick="goAng('+s+')">'+(unit?'按圈转':'按角度')+'</button>'+
  '</div>';
}
function motorCardHTML(s,unit,extra){
  return '<div class="mcard"><div class="mhead"><span class="name">'+MOT[s].n+'</span>'+
    '<span class="id">'+MOT[s].s+'</span><span class="led" id="led'+s+'"></span></div>'+
    motorLiveHTML(s)+(extra||"")+
    '<input type="range" id="sl'+s+'" min="-300" max="300" value="0" oninput="slider('+s+')">'+
    '<div class="sub" id="slv'+s+'">拖动滑块调速度</div>'+
    '<div class="row"><label>目标'+(unit||'角度 deg')+'</label>'+
    '<input type="number" id="ang'+s+'" step="'+(unit?'0.1':'1')+'" value="0">'+
    '<button class="ghost" onclick="goAng('+s+')">转到</button></div>'+
    '<div class="btns c3">'+
    '<button class="ghost" onclick="cmd(\'M,'+s+',0,0\')">停止</button>'+
    '<button class="ghost" onclick="cmd(\'Z,'+s+');toast(\'已设零\')">设零</button>'+
    (unit?'<button class="ghost" onclick="cmd(\'G,0\')">标准位</button>':'<button class="ghost" onclick="cmd(\'M,'+s+',2,0)">回零</button>')+
    '</div></div>';
}
function pidHTML(s){
  const rows=TF.map(f=>'<div class="row"><label>'+f[1]+'</label>'+
    '<input type="number" id="pf'+s+'_'+f[0]+'" step="'+f[3]+'" oninput="pidTouch('+s+')"></div>').join("");
  return '<details class="pid"><summary>'+MOT[s].n+' · PID 参数 <span class="tag" id="pidtag'+s+'">独立保存</span></summary>'+
    '<div class="body">'+rows+
    '<div class="btns c2" style="margin-top:10px">'+
    '<button onclick="applyPID('+s+')">应用并保存</button>'+
    '<button class="ghost" onclick="resetPID('+s+')">恢复默认</button>'+
    '</div></div></details>';
}
function buildStatic(){
  /* 界面1 电机卡 */
  $("cards").innerHTML=[0,1,2,3].map(s=>'<div class="mcard"><div class="mhead">'+
    '<span class="name">'+MOT[s].n+'</span><span class="id">'+MOT[s].s+'</span>'+
    '<span class="led" id="led'+s+'"></span></div>'+motorLiveHTML(s)+
    '<div class="btns c2"><button class="ghost" onclick="cmd(\'Z,'+s+');toast(\'已设零\')">设零</button>'+
    '<button class="ghost" onclick="cmd(\'M,'+s+',0,0)">停止</button></div></div>').join("");
  /* 界面2 拉簧 */
  $("spring0").innerHTML=motorCardHTML(0,"圈");$("spring1").innerHTML=motorCardHTML(1,"圈");
  $("springPid0").innerHTML=pidHTML(0);$("springPid1").innerHTML=pidHTML(1);
  /* 界面3 扳机 */
  $("trigCard").innerHTML=motorCardHTML(2,null);$("trigPid").innerHTML=pidHTML(2);
  /* 界面4 Yaw */
  $("yawCard").innerHTML=motorCardHTML(3,null);$("yawPid").innerHTML=pidHTML(3);
  /* 界面5 总控 */
  $("master").innerHTML=[0,1,2,3].map(s=>'<div class="mcard"><div class="mhead">'+
    '<span class="name">'+MOT[s].n+'</span><span class="id">'+MOT[s].s+'</span>'+
    '<span class="led" id="mled'+s+'"></span></div>'+
    '<div class="row"><label>转速</label><span class="v" id="mrpm'+s+'">0</span></div>'+
    '<div class="row"><label>速度 RPM</label><input type="number" id="ms'+s+'" step="1" value="0">'+
    '<button class="ghost" onclick="cmd(\'M,'+s+',1,\'+Math.round(ms'+s+'.value*10))">速度</button></div>'+
    '<div class="row"><label>角度 deg</label><input type="number" id="ma'+s+'" step="1" value="0">'+
    '<button class="ghost" onclick="cmd(\'M,'+s+',2,\'+Math.round(ma'+s+'.value*10))">角度</button>'+
    '<button class="ghost bad" onclick="cmd(\'M,'+s+',0,0)">停</button></div></div>').join("");
  /* 界面6 时间线 */
  $("timeline").innerHTML=AUTO.map((t,i)=>'<div class="step" id="st'+i+'"><span class="n">'+(i===0?"●":i)+'</span>'+t+'</div>').join("");
  renderDist();
}
function slider(s){const v=parseInt($("sl"+s).value);$("slv"+s).textContent=v+" RPM";cmd("M,"+s+",1,"+v*10);}
function goAng(s){const isTurns=(s===0||s===1);const v=parseFloat($("ang"+s).value||0);
  cmd("M,"+s+",2,"+Math.round(isTurns?v*3600:v*10));}
function setYaw(m){cmd("Y,"+m);}
function stopAll(){for(let s=0;s<4;s++)cmd("M,"+s+",0,0");toast("已全部停止");}
function runAllAngle(){for(let s=0;s<4;s++)goAng(s);toast("按角度运行");}
const pidDirty={};
function pidTouch(s){pidDirty[s]=1;}
function resetPID(s){pidDirty[s]=0;cmd("R,"+s);toast(MOT[s].n+" 已恢复默认");}
function applyPID(s){
  let i=0;
  const seq=TF.map(f=>[f[0],Math.round(parseFloat($("pf"+s+"_"+f[0]).value||0)*f[2])]);
  cmd("U,"+s);
  const next=()=>{if(i>=seq.length){pidDirty[s]=0;toast(MOT[s].n+" 参数已保存");return;}
    const q=seq[i++];cmd("P,"+q[0]+","+q[1]);setTimeout(next,35);};
  setTimeout(next,60);
}

/* ---------- 距离打击表 ---------- */
function loadDist(){try{return JSON.parse(localStorage.getItem("dartDist")||"null")||[[2,2],[2.5,2.5],[3,3],[3.5,3.5],[4,4]];}catch(e){return [];}}
function saveDist(d){localStorage.setItem("dartDist",JSON.stringify(d));}
function renderDist(){
  const d=loadDist();
  $("dgrid").innerHTML=d.map((r,i)=>'<button class="dbtn" onclick="fire('+i+')">'+r[0]+'m<small>'+r[1]+' 圈</small>'+
    '<span class="x" onclick="event.stopPropagation();delDist('+i+')">×</span></button>').join("")||'<div class="hint">还没有靶点，下面添加。</div>';
}
function addDist(){const d=parseFloat($("nd").value),t=parseFloat($("nt").value);
  if(isNaN(d)||isNaN(t)){toast("请填距离和圈数");return;}
  const a=loadDist();a.push([d,t]);a.sort((x,y)=>x[0]-y[0]);saveDist(a);renderDist();$("nd").value="";$("nt").value="";}
function delDist(i){const a=loadDist();a.splice(i,1);saveDist(a);renderDist();}
function toggleEdit(){editMode=!editMode;$("dgrid").className="dgrid"+(editMode?" edit":"");$("editBtn").textContent=editMode?"完成":"编辑";$("distTag").textContent=editMode?"点右上角 × 删除":"点击即发射";}
function fire(i){const r=loadDist()[i];if(!r)return;cmd("W,"+Math.round(r[1]*100));toast("发射 "+r[0]+"m · "+r[1]+"圈");setTimeout(()=>cmd("G,10"),180);}

/* ---------- 状态解析 ---------- */
function parse(t){
  if(!t||t[0]!=="F")return null;
  const k=t.split(",");const n=parseInt(k[1]);if(isNaN(n))return null;
  let i=2;const o={n:n,motors:[],params:[]};
  for(let j=0;j<n;j++){o.motors.push({slot:+k[i],type:+k[i+1],id:+k[i+2],online:+k[i+3],
    rpm:+k[i+4],pos:+k[i+5]/10,temp:+k[i+6],cur:+k[i+7]});i+=8;}
  o.sel=+k[i++];
  for(let j=0;j<n;j++){o.params.push(k.slice(i,i+11).map(Number));i+=11;}
  o.servo={cur:+k[i++],state:+k[i++],std:+k[i++],prep:+k[i++]};
  o.springTurns=+k[i++]/100;o.autoStep=+k[i++];o.yawMode=+k[i++];o.estop=+k[i++];
  o.vis={x:+k[i++],ok:+k[i++],center:+k[i++],err:+k[i++]};
  return o;
}

/* ---------- 刷新 ---------- */
function setTxt(id,v){const e=$(id);if(e&&e.textContent!==v)e.textContent=v;}
function update(){
  const now=Date.now();const live=(now-lastRx)<900;
  $("pill").className="pill"+(live?" ok":"");
  setTxt("pillTxt",live?"已连接":"未连接");
  setTxt("linkTxt",live?"正常":"无数据");
  setTxt("busTag",S.estop?"急停中":"CAN1 · 1Mbps");
  let on=0;
  S.motors.forEach(m=>{if(m.online)on++;});
  setTxt("onlineCnt",on+" / "+S.n);
  S.motors.forEach(m=>{
    const s=m.slot;
    setTxt("rpm"+s,m.rpm);setTxt("pos"+s,m.pos.toFixed(1));
    setTxt("tmp"+s,m.temp);setTxt("cur"+s,(m.cur/1000).toFixed(1));
    const l=$("led"+s);if(l)l.className="led"+(m.online?" on":"");
    const l2=$("mled"+s);if(l2)l2.className="led"+(m.online?" on":"");
    setTxt("mrpm"+s,m.rpm+" RPM");
  });
  /* 拉簧两卡显示名称 */
  /* PID 回填 (仅未编辑时) */
  S.params.forEach((p,s)=>{ if(!p||p.length<11||pidDirty[s])return;
    TF.forEach((f,idx)=>{const el=$("pf"+s+"_"+f[0]);
      if(el&&document.activeElement!==el) el.value=(p[idx]/f[2]).toFixed(3);});
  });
  /* 界面4 yaw */
  document.querySelectorAll("#yawSeg button").forEach(b=>b.className=(+b.dataset.m===S.yawMode?"on":""));
  const v=S.vis||{};
  setTxt("visTag",v.ok?"目标锁定":"丢失目标");
  setTxt("visX",v.x+" / "+v.center);setTxt("visErr",(v.err>0?"+":"")+v.err+" px");
  const mk=$("visMark");if(mk){let pct=50;if(v.center>0)pct=Math.max(2,Math.min(98,50+(v.err/v.center)*50));mk.style.left=pct+"%";}
  /* 界面6 时间线 */
  for(let i=1;i<=7;i++){const e=$("st"+i);if(!e)continue;
    e.className="step"+(S.autoStep===i?" on":(S.autoStep>i?" done":""));}
  setTxt("autoTag",AUTO[S.autoStep]||"空闲");
  /* 同步参数输入框 (仅当未聚焦) */
  [["mturns",S.springTurns],["aturns",S.springTurns],["astd",S.servo.std],["aprep",S.servo.prep]].forEach(([id,val])=>{
    const el=$(id);if(el&&document.activeElement!==el&&!el.dataset.touched)el.value=val;});
}

function poll(){
  fetch("/state").then(r=>r.text()).then(t=>{
    const o=parse(t);
    if(o){S=o;lastRx=Date.now();}
    update();
  }).catch(()=>update());
}
function doScan(){$("scanRes").textContent="扫描中...";fetch("/scan").catch(()=>{});setTimeout(()=>{
  fetch("/scanres").then(r=>r.text()).then(t=>{t=t.trim();
    $("scanRes").textContent=t?("CAN 反馈 ID: "+t):"未发现电机";}).catch(()=>{});},1400);}

buildTabs();
buildStatic();
[["mturns"],["aturns"],["astd"],["aprep"]].forEach(([id])=>{const el=$(id);if(el)el.addEventListener("input",()=>el.dataset.touched="1");});
setInterval(poll,300);
setInterval(hb,500);
poll();
</script>
</body>
</html>
"""


def send_all(sock, data):
    mv = memoryview(data)
    while len(mv):
        n = sock.send(mv)
        if n is None:
            break
        mv = mv[n:]


def http_ok(cl, ctype, body):
    hdr = ("HTTP/1.1 200 OK\r\nContent-Type: %s\r\nContent-Length: %d\r\n"
           "Connection: close\r\n\r\n" % (ctype, len(body)))
    send_all(cl, hdr.encode())
    send_all(cl, body)


def parse_query(path):
    q = {}
    if "?" in path:
        for kv in path.split("?", 1)[1].split("&"):
            if "=" in kv:
                k, v = kv.split("=", 1)
                q[k] = v
    return q


def urldecode(s):
    out = bytearray()
    i = 0
    n = len(s)
    while i < n:
        c = s[i]
        if c == "%" and i + 2 < n:
            try:
                out.append(int(s[i + 1:i + 3], 16))
                i += 3
                continue
            except Exception:
                pass
        if c == "+":
            out.append(32)
        else:
            out.append(ord(c))
        i += 1
    return out.decode()


uart = UART(UART_ID, baudrate=BAUD, tx=UART_TX, rx=UART_RX)

ap = network.WLAN(network.AP_IF)
ap.active(True)
ap.config(essid=AP_SSID, password=AP_PASS, max_clients=4)
while not ap.active():
    time.sleep_ms(50)
print("AP ready:", ap.ifconfig()[0])

srv = socket.socket()
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind(socket.getaddrinfo("0.0.0.0", 80)[0][-1])
srv.listen(4)
srv.settimeout(0.05)
print("HTTP server listening on :80")

state_str = ""
scan_ids = []
rxbuf = b""


while True:
    while True:
        try:
            cl, _ = srv.accept()
        except OSError:
            break
        try:
            cl.settimeout(0.3)
            req = cl.recv(1024).decode()
            path = req.split(" ", 2)[1] if " " in req else "/"
            if path.startswith("/cmd"):
                q = parse_query(path)
                if "c" in q:
                    try:
                        uart.write(urldecode(q["c"]) + "\n")
                    except Exception:
                        pass
                http_ok(cl, "text/plain", b"ok")
            elif path.startswith("/ping"):
                try:
                    uart.write("H\n")
                except Exception:
                    pass
                http_ok(cl, "text/plain", b"ok")
            elif path.startswith("/scanres"):
                http_ok(cl, "text/plain", ",".join(str(x) for x in scan_ids).encode())
            elif path.startswith("/scan"):
                try:
                    uart.write("S\n")
                except Exception:
                    pass
                http_ok(cl, "text/plain", b"ok")
            elif path.startswith("/state"):
                http_ok(cl, "text/plain", state_str.encode())
            elif path.startswith("/favicon"):
                http_ok(cl, "image/x-icon", b"")
            else:
                http_ok(cl, "text/html; charset=utf-8", PAGE.encode())
        except Exception as e:
            print("http err:", e)
        finally:
            try:
                cl.close()
            except Exception:
                pass

    try:
        rx = uart.read()
        if rx:
            rxbuf += rx
            while b"\n" in rxbuf:
                line, rxbuf = rxbuf.split(b"\n", 1)
                line = line.strip()
                if line.startswith(b"F,"):
                    state_str = line.decode()
                elif line.startswith(b"S,"):
                    p = line.decode().split(",")
                    if len(p) >= 2:
                        n = int(p[1])
                        scan_ids = [int(x) for x in p[2:2 + n]]
    except Exception:
        pass

    time.sleep_ms(10)
