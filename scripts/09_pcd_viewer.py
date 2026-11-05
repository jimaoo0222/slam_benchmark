#!/usr/bin/env python3
"""PCD / 点云查看器 —— 双击打开，把 .pcd 拖进浏览器就能转着看。

用法
────
    python3 scripts/09_pcd_viewer.py                 # 启动并自动开浏览器
    python3 scripts/09_pcd_viewer.py --no-browser    # 只起服务，自己开
    python3 scripts/09_pcd_viewer.py --port 8800

能拖什么
────────
    *.pcd         点云（本项目 06_save_map_pcd.py 写的 binary PCD，也支持 ascii）
    *.txt         轨迹（TUM 格式：timestamp tx ty tz qx qy qz qw，可以叠加到点云上）
    *.ply         （可选）如果装了 plyfile 才支持

为什么不用桌面 GUI
──────────────────
PyQt + OpenGL 要装一堆依赖，而且 6 GB 显存跑 70 万点也未必流畅。
浏览器 + WebGL 零依赖、开箱即用，还能随手分享给别人。
plotly.js 从本机 plotly 包里读（4.3 MB），完全离线。
"""
from __future__ import annotations

import argparse
import http.server
import importlib.util
import socket
import socketserver
import sys
import threading
import webbrowser
from pathlib import Path

HTML = r"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<title>PCD 点云查看器</title>
<style>
  :root { --bg:#0e0e12; --panel:#17171e; --line:#2a2a38; --fg:#dcdce6; --dim:#84849a; --acc:#4ea1ff; }
  * { box-sizing: border-box; }
  html, body { margin:0; height:100%; overflow:hidden;
    background:var(--bg); color:var(--fg);
    font:13px/1.5 "Segoe UI","Noto Sans CJK SC",system-ui,sans-serif; }
  #app { display:flex; height:100%; }
  #side { width:250px; flex:0 0 250px; background:var(--panel);
    border-right:1px solid var(--line); padding:14px; overflow-y:auto; }
  #side h1 { font-size:14px; margin:0 0 4px; letter-spacing:.3px; }
  #side .sub { color:var(--dim); font-size:11px; margin-bottom:14px; }
  .grp { margin-bottom:16px; }
  .grp label { display:block; color:var(--dim); font-size:11px;
    text-transform:uppercase; letter-spacing:.6px; margin-bottom:6px; }
  select, input[type=range] { width:100%; }
  select { background:#22222c; color:var(--fg); border:1px solid var(--line);
    border-radius:5px; padding:5px 7px; font-size:12px; }
  input[type=range] { accent-color:var(--acc); }
  .row { display:flex; justify-content:space-between; align-items:center;
    font-size:11px; color:var(--dim); margin-top:3px; }
  .chk { display:flex; align-items:center; gap:7px; color:var(--fg);
    font-size:12px; margin:7px 0; cursor:pointer; }
  .chk input { accent-color:var(--acc); }
  #info { border-top:1px solid var(--line); padding-top:12px; font-size:11px;
    color:var(--dim); white-space:pre-wrap; line-height:1.7; }
  #info b { color:var(--fg); font-weight:600; }
  #main { flex:1; position:relative; }
  #plot { position:absolute; inset:0; }
  #drop { position:absolute; inset:0; display:flex; flex-direction:column;
    align-items:center; justify-content:center; gap:14px;
    background:rgba(14,14,18,.94); z-index:10; text-align:center; padding:30px; }
  #drop.hide { display:none; }
  #drop .big { font-size:52px; opacity:.35; line-height:1; }
  #drop h2 { margin:0; font-size:17px; font-weight:600; }
  #drop p { margin:0; color:var(--dim); font-size:12.5px; max-width:440px; line-height:1.8; }
  #drop code { background:#22222c; padding:2px 6px; border-radius:4px; color:var(--acc); }
  body.dragover #drop { background:rgba(78,161,255,.12); }
  body.dragover #drop h2 { color:var(--acc); }
  #toast { position:absolute; bottom:18px; left:50%; transform:translateX(-50%);
    background:#22222c; border:1px solid var(--line); border-radius:7px;
    padding:9px 18px; font-size:12px; opacity:0; transition:opacity .25s;
    pointer-events:none; z-index:20; max-width:80%; }
  #toast.show { opacity:1; }
  #toast.err { border-color:#c0392b; color:#ff9a8f; }
  #files { font-size:11px; color:var(--dim); margin-top:6px; word-break:break-all; }
  #files .f { color:var(--acc); }
  .btn { width:100%; background:#22222c; color:var(--fg); border:1px solid var(--line);
    border-radius:5px; padding:6px; font-size:12px; cursor:pointer; margin-top:6px; }
  .btn:hover { background:#2c2c3a; border-color:#3a3a4e; }
</style>
</head>
<body>
<div id="app">
  <div id="side">
    <h1>点云查看器</h1>
    <div class="sub">拖 .pcd / .txt 进来</div>

    <div class="grp">
      <label>着色</label>
      <select id="colorMode">
        <option value="z">按高度 z</option>
        <option value="intensity">按反射率</option>
        <option value="range">按距离</option>
        <option value="flat">单色</option>
      </select>
    </div>

    <div class="grp">
      <label>点大小</label>
      <input type="range" id="ptSize" min="0.4" max="4" step="0.1" value="1.2">
      <div class="row"><span>细</span><span id="ptSizeVal">1.2</span></div>
    </div>

    <div class="grp">
      <label>最多渲染点数</label>
      <select id="target">
        <option value="30000">3 万（最快）</option>
        <option value="60000" selected>6 万（推荐）</option>
        <option value="120000">12 万</option>
        <option value="300000">30 万（可能卡）</option>
      </select>
      <div class="row"><span>超过会体素降采样</span></div>
    </div>

    <div class="grp">
      <label>显示</label>
      <label class="chk"><input type="checkbox" id="showTraj" checked> 轨迹</label>
      <label class="chk"><input type="checkbox" id="showAxes" checked> 坐标轴</label>
    </div>

    <div class="grp">
      <button class="btn" id="reset">复位视角</button>
      <button class="btn" id="clear">清空</button>
    </div>

    <div id="info">还没有数据。</div>
  </div>

  <div id="main">
    <div id="plot"></div>
    <div id="drop">
      <div class="big">⬇</div>
      <h2>把点云文件拖进来</h2>
      <p>支持 <code>.pcd</code>（binary / ascii）和 <code>.txt</code>（TUM 轨迹）。<br>
         同时拖一个 pcd + 一个 txt → 点云上叠加轨迹。<br><br>
         也可以点下面的按钮选文件。</p>
      <button class="btn" style="width:auto;padding:8px 22px" id="pick">选择文件…</button>
      <input type="file" id="fileInput" multiple accept=".pcd,.txt" style="display:none">
    </div>
    <div id="toast"></div>
  </div>
</div>

<script src="/plotly.min.js"></script>
<script>
"use strict";
let cloud = null;      // {xyz:Float32Array, inten:Float32Array, n}
let cloudRaw = 0;      // 原始点数
let traj = null;       // {xyz:Float32Array, n}
let cloudName = "", trajName = "";

const $ = id => document.getElementById(id);
const plotDiv = $("plot");
let plotReady = false;

function toast(msg, isErr) {
  const t = $("toast");
  t.textContent = msg;
  t.className = "show" + (isErr ? " err" : "");
  clearTimeout(t._h);
  t._h = setTimeout(() => { t.className = ""; }, 3800);
}

/* ───────────── PCD 解析（binary + ascii） ───────────── */
function parsePCD(buf) {
  const bytes = new Uint8Array(buf);
  // header 是 ASCII，最多扫前 256 KB
  const head = new TextDecoder("ascii").decode(bytes.subarray(0, Math.min(bytes.length, 262144)));
  const nl = head.search(/\nDATA\s+(\w+)\s*\n/);
  if (nl < 0) throw new Error("不是合法的 PCD（找不到 DATA 行）");
  const mData = head.slice(nl).match(/\nDATA\s+(\w+)\s*\n/);
  const mode = mData[1].toLowerCase();
  const headText = head.slice(0, nl + mData[0].length);

  const get = k => {
    const m = headText.match(new RegExp("^" + k + "\\s+(.+)$", "mi"));
    return m ? m[1].trim() : "";
  };
  const fields = get("FIELDS").split(/\s+/).filter(Boolean);
  const sizes = get("SIZE").split(/\s+/).map(Number);
  const types = get("TYPE").split(/\s+/).map(s => s.toUpperCase());
  const counts = get("COUNT").split(/\s+/).map(Number);
  if (!fields.length || !sizes.length) throw new Error("PCD header 缺 FIELDS/SIZE/TYPE");
  const nPts = parseInt(get("POINTS"), 10) || 0;

  const ix = fields.indexOf("x"), iy = fields.indexOf("y"), iz = fields.indexOf("z");
  if (ix < 0 || iy < 0 || iz < 0) throw new Error("点云里没有 x/y/z 字段");
  let iInt = fields.indexOf("intensity");
  if (iInt < 0) iInt = fields.indexOf("reflectivity");

  const out = { xyz: null, inten: null, n: 0 };

  if (mode === "ascii") {
    const bodyStart = new TextEncoder().encode(headText).length;
    const body = new TextDecoder("ascii").decode(bytes.subarray(bodyStart));
    const lines = body.split("\n");
    const xs = [], ys = [], zs = [], ii = [];
    const ci = counts.length ? counts : fields.map(() => 1);
    for (const line of lines) {
      const t = line.trim();
      if (!t || t[0] === "#") continue;
      const p = t.split(/\s+/);
      if (p.length < fields.length) continue;
      xs.push(+p[ix]); ys.push(+p[iy]); zs.push(+p[iz]);
      if (iInt >= 0 && iInt < p.length) ii.push(+p[iInt]);
    }
    out.n = xs.length;
    out.xyz = new Float32Array(out.n * 3);
    out.inten = new Float32Array(out.n);
    for (let k = 0; k < out.n; k++) {
      out.xyz[k*3] = xs[k]; out.xyz[k*3+1] = ys[k]; out.xyz[k*3+2] = zs[k];
      out.inten[k] = iInt >= 0 ? (ii[k] || 0) : 0;
    }
    return out;
  }

  if (mode !== "binary") throw new Error("暂不支持 DATA " + mode + "（只支持 binary / ascii）");

  // binary：按 field 的 offset 读
  const wrapLen = new TextEncoder().encode(headText).length;
  const view = new DataView(buf, wrapLen);
  // 每点字节数
  let step = 0;
  const offsets = [];
  for (let f = 0; f < fields.length; f++) {
    offsets.push(step);
    step += sizes[f] * (counts[f] || 1);
  }
  const nAvail = Math.floor(view.byteLength / step);
  const n = nPts > 0 ? Math.min(nPts, nAvail) : nAvail;

  out.n = n;
  out.xyz = new Float32Array(n * 3);
  out.inten = new Float32Array(n);
  for (let k = 0; k < n; k++) {
    const base = k * step;
    out.xyz[k*3]   = readNum(view, base + offsets[ix], sizes[ix], types[ix]);
    out.xyz[k*3+1] = readNum(view, base + offsets[iy], sizes[iy], types[iy]);
    out.xyz[k*3+2] = readNum(view, base + offsets[iz], sizes[iz], types[iz]);
    out.inten[k] = iInt >= 0 ? readNum(view, base + offsets[iInt], sizes[iInt], types[iInt]) : 0;
  }
  return out;
}

function readNum(view, off, size, type) {
  if (type === "F") return size === 8 ? view.getFloat64(off, true) : view.getFloat32(off, true);
  if (type === "I") return size === 1 ? view.getInt8(off) : size === 2 ? view.getInt16(off, true) : view.getInt32(off, true);
  return size === 1 ? view.getUint8(off) : size === 2 ? view.getUint16(off, true) : view.getUint32(off, true);
}

/* ───────────── TUM 轨迹 ───────────── */
function parseTUM(text) {
  const xs = [], ys = [], zs = [];
  for (const line of text.split("\n")) {
    const t = line.trim();
    if (!t || t[0] === "#") continue;
    const p = t.split(/\s+/);
    if (p.length < 4) continue;
    const x = +p[1], y = +p[2], z = +p[3];
    if (!isFinite(x) || !isFinite(y) || !isFinite(z)) continue;
    xs.push(x); ys.push(y); zs.push(z);
  }
  const n = xs.length;
  if (!n) return null;
  const xyz = new Float32Array(n * 3);
  for (let k = 0; k < n; k++) { xyz[k*3] = xs[k]; xyz[k*3+1] = ys[k]; xyz[k*3+2] = zs[k]; }
  return { xyz, n };
}

/* ───────────── 体素降采样 ───────────── */
function voxelDownsample(c, target) {
  if (!c || c.n <= target) return c;
  const { xyz } = c;
  let lo = 0.005, hi = 8.0, best = c;
  for (let it = 0; it < 22; it++) {
    const v = (lo + hi) / 2;
    const seen = new Set();
    const keep = [];
    for (let k = 0; k < c.n; k++) {
      const gx = Math.floor(xyz[k*3] / v), gy = Math.floor(xyz[k*3+1] / v), gz = Math.floor(xyz[k*3+2] / v);
      const key = gx + "," + gy + "," + gz;
      if (!seen.has(key)) { seen.add(key); keep.push(k); }
    }
    const n = keep.length;
    const nxyz = new Float32Array(n * 3), nint = new Float32Array(n);
    for (let j = 0; j < n; j++) {
      const k = keep[j];
      nxyz[j*3] = xyz[k*3]; nxyz[j*3+1] = xyz[k*3+1]; nxyz[j*3+2] = xyz[k*3+2];
      nint[j] = c.inten[k];
    }
    best = { xyz: nxyz, inten: nint, n };
    if (n > target) lo = v; else hi = v;
  }
  return best;
}

/* ───────────── 渲染 ───────────── */
function colorArray(c) {
  const mode = $("colorMode").value;
  const n = c.n;
  const out = new Array(n);
  if (mode === "z") {
    for (let k = 0; k < n; k++) out[k] = c.xyz[k*3+2];
  } else if (mode === "intensity") {
    for (let k = 0; k < n; k++) out[k] = c.inten[k];
  } else if (mode === "range") {
    for (let k = 0; k < n; k++) {
      const x = c.xyz[k*3], y = c.xyz[k*3+1], z = c.xyz[k*3+2];
      out[k] = Math.sqrt(x*x + y*y + z*z);
    }
  } else {
    for (let k = 0; k < n; k++) out[k] = 0;
  }
  return out;
}

function render() {
  if (!cloud) { Plotly.purge(plotDiv); plotReady = false; return; }
  const target = +$("target").value;
  const shown = voxelDownsample(cloud, target);
  const xs = new Array(shown.n), ys = new Array(shown.n), zs = new Array(shown.n);
  for (let k = 0; k < shown.n; k++) {
    xs[k] = shown.xyz[k*3]; ys[k] = shown.xyz[k*3+1]; zs[k] = shown.xyz[k*3+2];
  }
  const mode = $("colorMode").value;
  const traces = [];

  traces.push({
    type: "scatter3d", mode: "markers", name: "map",
    x: xs, y: ys, z: zs,
    marker: {
      size: +$("ptSize").value,
      color: mode === "flat" ? "#6cb6ff" : colorArray(shown),
      colorscale: "Viridis",
      opacity: 0.85,
      colorbar: mode === "flat" ? undefined : {
        title: { text: mode === "z" ? "z (m)" : mode === "intensity" ? "intensity" : "range (m)", side: "right" },
        thickness: 12, len: 0.6, x: 1.0,
      },
    },
    hovertemplate: "x=%{x:.2f} y=%{y:.2f} z=%{z:.2f}<extra></extra>",
  });

  if (traj && $("showTraj").checked) {
    const tx = [], ty = [], tz = [];
    for (let k = 0; k < traj.n; k++) { tx.push(traj.xyz[k*3]); ty.push(traj.xyz[k*3+1]); tz.push(traj.xyz[k*3+2]); }
    traces.push({ type:"scatter3d", mode:"lines", name:"trajectory",
      x:tx, y:ty, z:tz, line:{ color:"#ff3b30", width:6 } });
    traces.push({ type:"scatter3d", mode:"markers+text", name:"start", text:["START"], textposition:"top center",
      x:[tx[0]], y:[ty[0]], z:[tz[0]], marker:{ size:6, color:"#28d17c" }, textfont:{ color:"#28d17c", size:11 } });
    traces.push({ type:"scatter3d", mode:"markers+text", name:"end", text:["END"], textposition:"top center",
      x:[tx[tx.length-1]], y:[ty[ty.length-1]], z:[tz[tz.length-1]], marker:{ size:6, color:"#ff3b30" }, textfont:{ color:"#ff3b30", size:11 } });
  }

  const layout = {
    paper_bgcolor: "#0e0e12", plot_bgcolor: "#0e0e12",
    font: { color: "#c8c8d6", size: 11 },
    margin: { l:0, r:0, t:0, b:0 },
    showlegend: true,
    legend: { x:0.01, y:0.99, bgcolor:"rgba(23,23,30,.85)", bordercolor:"#2a2a38", borderwidth:1, font:{size:10} },
    scene: {
      bgcolor: "#0e0e12",
      aspectmode: "data",
      xaxis: { title:"x (m)", gridcolor:"#26263a", zerolinecolor:"#3a3a52", color:"#9a9ab0",
               showbackground: $("showAxes").checked, backgroundcolor:"#131319" },
      yaxis: { title:"y (m)", gridcolor:"#26263a", zerolinecolor:"#3a3a52", color:"#9a9ab0",
               showbackground: $("showAxes").checked, backgroundcolor:"#131319" },
      zaxis: { title:"z (m)", gridcolor:"#26263a", zerolinecolor:"#3a3a52", color:"#9a9ab0",
               showbackground: $("showAxes").checked, backgroundcolor:"#131319" },
      camera: { eye: { x:-1.55, y:-1.55, z:0.85 }, up:{x:0,y:0,z:1} },
    },
  };
  const config = { scrollZoom:true, displaylogo:false,
    modeBarButtonsToRemove:["toImage","sendDataToCloud","select2d","lasso2d"] };

  if (!plotReady) {
    Plotly.newPlot(plotDiv, traces, layout, config);
    plotReady = true;
  } else {
    Plotly.react(plotDiv, traces, layout, config);
  }
  if (shown.n) $("drop").classList.add("hide");
  updateInfo(shown);
}

function updateInfo(shown) {
  const c = cloud;
  let s = "";
  s += "<b>点云</b> " + cloudName + "\n";
  s += "  原始 " + cloudRaw.toLocaleString() + " 点\n";
  if (shown && shown.n !== cloudRaw) s += "  渲染 " + shown.n.toLocaleString() + " 点（已降采样）\n";
  let x0=1e9,x1=-1e9,y0=1e9,y1=-1e9,z0=1e9,z1=-1e9;
  for (let k = 0; k < c.n; k++) {
    const x=c.xyz[k*3], y=c.xyz[k*3+1], z=c.xyz[k*3+2];
    if (x<x0)x0=x; if (x>x1)x1=x; if (y<y0)y0=y; if (y>y1)y1=y; if (z<z0)z0=z; if (z>z1)z1=z;
  }
  s += "  x " + x0.toFixed(1) + " ~ " + x1.toFixed(1) + " m\n";
  s += "  y " + y0.toFixed(1) + " ~ " + y1.toFixed(1) + " m\n";
  s += "  z " + z0.toFixed(1) + " ~ " + z1.toFixed(1) + " m\n";
  if (traj) s += "\n<b>轨迹</b> " + trajName + "\n  " + traj.n + " 个位姿\n";
  s += "\n左键拖动=旋转\n滚轮=缩放\n右键拖动=平移";
  $("info").innerHTML = s;
}

/* ───────────── 文件载入 ───────────── */
async function loadFiles(files) {
  const arr = Array.from(files);
  if (!arr.length) return;
  $("drop").classList.add("hide");
  toast("读取中…");
  for (const f of arr) {
    const low = f.name.toLowerCase();
    try {
      if (low.endsWith(".pcd")) {
        const buf = await f.arrayBuffer();
        const c = parsePCD(buf);
        if (!c.n) throw new Error("0 个点，文件可能是空的");
        cloud = c; cloudRaw = c.n; cloudName = f.name;
      } else if (low.endsWith(".txt") || low.endsWith(".tum") || low.endsWith(".csv")) {
        const text = await f.text();
        const t = parseTUM(text);
        if (!t) throw new Error("没读到轨迹点（需要 TUM 8 列格式）");
        traj = t; trajName = f.name;
      } else {
        throw new Error("不支持的文件类型：" + low.split(".").pop());
      }
    } catch (e) {
      toast(f.name + "：" + e.message, true);
      console.error(e);
      return;
    }
  }
  render();
  if (cloud) toast("✅ " + cloudRaw.toLocaleString() + " 点" + (traj ? " + 轨迹 " + traj.n : ""));
  else if (traj) toast("✅ 轨迹 " + traj.n + " 位姿");
}

/* ───────────── 事件 ───────────── */
["dragenter","dragover"].forEach(ev =>
  document.addEventListener(ev, e => { e.preventDefault(); document.body.classList.add("dragover"); }));
["dragleave","drop"].forEach(ev =>
  document.addEventListener(ev, e => {
    e.preventDefault();
    if (ev === "drop" || e.target === document.documentElement || !e.relatedTarget)
      document.body.classList.remove("dragover");
  }));
document.addEventListener("drop", e => {
  if (e.dataTransfer && e.dataTransfer.files.length) loadFiles(e.dataTransfer.files);
});

$("pick").onclick = () => $("fileInput").click();
$("fileInput").onchange = e => { loadFiles(e.target.files); e.target.value = ""; };
$("colorMode").onchange = render;
$("ptSize").oninput = e => { $("ptSizeVal").textContent = e.target.value; render(); };
$("target").onchange = render;
$("showTraj").onchange = render;
$("showAxes").onchange = render;
$("reset").onclick = () => { if (plotReady) Plotly.relayout(plotDiv,
  { "scene.camera": { eye:{x:-1.55,y:-1.55,z:0.85}, up:{x:0,y:0,z:1} } }); };
$("clear").onclick = () => {
  cloud = traj = null; cloudName = trajName = ""; cloudRaw = 0;
  Plotly.purge(plotDiv); plotReady = false;
  $("drop").classList.remove("hide");
  $("info").textContent = "还没有数据。";
};

/* ───────────── 启动时按 URL 里的 ?open= 自动载入 ───────────── */
(async function autoOpen() {
  const q = new URLSearchParams(location.search);
  const list = q.getAll("open");
  if (!list.length) return;
  $("drop").classList.add("hide");
  toast("载入中…");
  for (const path of list) {
    const name = path.split("/").pop();
    try {
      const r = await fetch("/api/read?path=" + encodeURIComponent(path));
      if (!r.ok) throw new Error("读取失败（" + r.status + "）");
      const low = name.toLowerCase();
      if (low.endsWith(".pcd")) {
        const c = parsePCD(await r.arrayBuffer());
        cloud = c; cloudRaw = c.n; cloudName = name;
      } else {
        const t = parseTUM(await r.text());
        if (!t) throw new Error("不是 TUM 轨迹");
        traj = t; trajName = name;
      }
    } catch (e) { toast(name + "：" + e.message, true); return; }
  }
  render();
  if (cloud) toast("✅ " + cloudRaw.toLocaleString() + " 点" + (traj ? " + 轨迹 " + traj.n : ""));
})();
</script>
</body>
</html>
"""


def find_port(preferred: int, tries: int = 40) -> int:
    for p in range(preferred, preferred + tries):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                s.bind(("127.0.0.1", p))
                return p
            except OSError:
                continue
    raise SystemExit(f"❌ {preferred}~{preferred+tries} 都被占用，用 --port 换一个")


def plotly_js() -> bytes | None:
    """从本机 plotly 包里读 plotly.min.js（离线可用，不依赖 CDN）。"""
    spec = importlib.util.find_spec("plotly")
    if spec and spec.origin:
        candidate = Path(spec.origin).parent / "package_data" / "plotly.min.js"
        if candidate.is_file():
            return candidate.read_bytes()
    for base in (
        Path.home() / "anaconda3/lib/python3.13/site-packages/plotly/package_data/plotly.min.js",
        Path("/usr/lib/python3/dist-packages/plotly/package_data/plotly.min.js"),
    ):
        if base.is_file():
            return base.read_bytes()
    return None


def make_handler(js: bytes | None):
    class Handler(http.server.BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):  # 安静点
            pass

        def _send(self, body: bytes, ctype: str, code: int = 200) -> None:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path_only = self.path.split("?", 1)[0]
            if path_only in ("/", "/index.html"):
                self._send(HTML.encode("utf-8"), "text/html; charset=utf-8")
            elif path_only == "/plotly.min.js":
                if js is None:
                    self._send(b"console.error('plotly.min.js not found');",
                               "application/javascript", 404)
                else:
                    self._send(js, "application/javascript")
            elif self.path.startswith("/api/read?"):
                # 供「双击某个 .pcd 用本程序打开」用：只允许 pcd/txt，且只监听本机
                from urllib.parse import parse_qs, unquote
                qs = parse_qs(self.path.split("?", 1)[1])
                raw = unquote(qs.get("path", [""])[0])
                p = Path(raw).expanduser()
                if p.suffix.lower() not in (".pcd", ".txt", ".tum", ".csv"):
                    self._send(b"unsupported", "text/plain", 403)
                elif not p.is_file():
                    self._send(b"not found", "text/plain", 404)
                else:
                    self._send(p.read_bytes(), "application/octet-stream")
            elif self.path == "/favicon.ico":
                self._send(b"", "image/x-icon", 204)
            else:
                self._send(b"not found", "text/plain", 404)

    return Handler


def main() -> int:
    ap = argparse.ArgumentParser(description="PCD 点云查看器（本地网页版）")
    ap.add_argument("--port", type=int, default=8765, help="起始端口（默认 8765）")
    ap.add_argument("--no-browser", action="store_true", help="不自动开浏览器")
    ap.add_argument("files", nargs="*", help="可选：启动时直接打开的 .pcd / .txt")
    args = ap.parse_args()

    js = plotly_js()
    if js is None:
        print("⚠️ 找不到 plotly.min.js —— 页面里的 3D 图不会显示")
        print("   修：python3 -m pip install --user plotly")
    else:
        print(f"✅ plotly.js 已就绪（{len(js)/1048576:.1f} MB，离线可用）")

    port = find_port(args.port)
    from urllib.parse import quote
    query = ""
    if args.files:
        parts = []
        for f in args.files:
            p = Path(f).expanduser().resolve()
            if not p.is_file():
                print(f"⚠️ 跳过不存在的文件: {f}")
                continue
            parts.append("open=" + quote(str(p)))
        if parts:
            query = "?" + "&".join(parts)
    url = f"http://127.0.0.1:{port}/{query}"

    class Server(socketserver.ThreadingMixIn, http.server.HTTPServer):
        daemon_threads = True
        allow_reuse_address = True

    srv = Server(("127.0.0.1", port), make_handler(js))

    print("=" * 62)
    print(" 点云查看器")
    print("=" * 62)
    print(f"   地址: {url}")
    print("   拖 .pcd（点云）或 .txt（TUM 轨迹）进浏览器窗口")
    print("   左键拖动=旋转  滚轮=缩放  右键拖动=平移")
    print()
    print("   关掉这个终端（或 Ctrl+C）就退出")
    print("=" * 62)

    if not args.no_browser:
        threading.Timer(0.6, lambda: webbrowser.open(url)).start()

    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\n已退出")
    finally:
        srv.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
