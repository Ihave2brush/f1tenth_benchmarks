'use strict';
const el = id => document.getElementById(id);
const canvas = el('canvas'), ctx = canvas.getContext('2d');
const colors = ['#49d8f0', '#ffb45c', '#71e4a7', '#f58db2', '#bda1ff'];
const fmt = (n, digits = 3) => n == null ? '—' : Number(n).toFixed(digits);
let data, image, frame, cursor = 0, playing = false, loading = false, timer, playEpoch = 0;
let requestSequence = 0, pending, scale = 1, ox = 0, oy = 0, drag;
let anchorWall = 0, anchorSim = 0;
const endNames = {requested_laps_reached:'已完成指定圈數', control_step_limit:'短預覽結束（未完成整圈）', timeout:'模擬逾時', collision:'模擬碰撞', error:'執行中斷', projection_invalid:'定位失敗', tracking_limit_exceeded:'追蹤誤差超標'};

async function api(path, signal) {
  const response = await fetch(path, {signal});
  const result = await response.json();
  if (!response.ok) throw Error(result.error || '無法讀取記錄');
  return result;
}
function values(id, pairs) {
  el(id).replaceChildren();
  for (const [label, text] of pairs) {
    const dt = document.createElement('dt'), dd = document.createElement('dd');
    dt.textContent = label; dd.textContent = text; el(id).append(dt, dd);
  }
}
function line(points, color, width = 1.5, dashed = false, close = false) {
  ctx.strokeStyle = color; ctx.lineWidth = width / scale;
  ctx.setLineDash(dashed ? [5 / scale, 4 / scale] : []);
  ctx.beginPath(); let started = false;
  for (const p of points) {
    if (!p) { started = false; continue; }
    if (started) ctx.lineTo(...p); else ctx.moveTo(...p);
    started = true;
  }
  if (close) ctx.closePath(); ctx.stroke(); ctx.setLineDash([]);
}
function dot(p, color, size = 3) {
  if (!p) return;
  ctx.fillStyle = color; ctx.beginPath(); ctx.arc(...p, size / scale, 0, Math.PI * 2); ctx.fill();
}
function centerCar(zoom = false) {
  if (!frame || !frame.pixel) return;
  if (zoom) scale = Math.max(scale, 3.2);
  ox = canvas.width / 2 - frame.pixel[0] * scale;
  oy = canvas.height / 2 - frame.pixel[1] * scale;
}
function fit() {
  if (!data) return;
  scale = Math.min(canvas.width / data.width, canvas.height / data.height) * .9;
  ox = (canvas.width - data.width * scale) / 2; oy = (canvas.height - data.height * scale) / 2;
  el('follow').checked = false; draw();
}
function draw() {
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  if (!data || !image) return;
  ctx.save(); ctx.translate(ox, oy); ctx.scale(scale, scale); ctx.drawImage(image, 0, 0);
  if (el('center').checked) line(data.center, '#3aa3b5', 1.1);
  if (el('bounds').checked) {
    data.left.forEach(p => dot(p, '#dc8990', .8));
    data.right.forEach(p => dot(p, '#7dacf3', .8));
  }
  if (frame) {
    if (el('trail').checked) line(frame.trail, '#ffd86c', 1.8);
    if (frame.plan && el('paths').checked) {
      for (const p of frame.plan.paths) {
        line(p.pixels, colors[p.candidate_id - 1], 2.4, !p.sampled_valid);
        if (p.first_invalid_pixel) {
          const [x, y] = p.first_invalid_pixel, d = 4 / scale;
          line([[x-d,y-d],[x+d,y+d]], '#ff6d5f', 2);
          line([[x-d,y+d],[x+d,y-d]], '#ff6d5f', 2);
        }
      }
    }
    if (frame.plan && el('source').checked) {
      line(frame.plan.source_body, '#ffffff', 1.6, true, true);
      dot(frame.plan.source_pixel, '#ffffff', 2.2);
    }
    if (frame.body.length) {
      ctx.fillStyle = '#f5f8fa'; ctx.beginPath();
      frame.body.forEach((p,i) => i ? ctx.lineTo(...p) : ctx.moveTo(...p)); ctx.closePath(); ctx.fill();
      line(frame.body, '#20333c', 1.2, false, true);
      if (frame.heading_pixel) line([frame.pixel, frame.heading_pixel], '#f2755a', 2.4);
    }
  }
  ctx.restore();
}
function resize() {
  canvas.width = canvas.clientWidth; canvas.height = canvas.clientHeight;
  if (el('follow').checked) { centerCar(); draw(); } else fit();
}
function updateStatus() {
  if (!frame) return;
  el('play').textContent = playing ? '暫停' : '播放';
  el('previous').disabled = cursor === 0; el('next').disabled = frame.at_end;
  el('play').disabled = (!playing && loading) || (frame.at_end && !playing);
  el('playbackStatus').textContent = frame.at_end ? '已到結尾 · ' + (endNames[frame.end_reason] || frame.end_reason) : playing ? '播放中 · ' + el('rate').value + '×' : '已暫停';
}
function pause() {
  playing = false; ++playEpoch; clearTimeout(timer); updateStatus();
}
function apply(next) {
  frame = next; cursor = frame.control_step;
  el('timeline').value = cursor; el('stepInput').value = cursor;
  el('timeInput').value = fmt(frame.relative_time_s, 3);
  el('position').textContent = `步號 ${cursor} / ${data.state_count - 1}`;
  el('timeLabel').textContent = `${fmt(frame.relative_time_s, 2)} / ${fmt(data.duration_s, 2)} s`;
  el('stateStatus').textContent = frame.valid ? '實際記錄車況 · ' + (frame.at_end ? '最後一筆' : '第 ' + cursor + ' 步') : '車況無效：' + frame.reason;
  const f = frame.frenet;
  values('stateValues', [['速度 m/s',fmt(frame.speed_mps)], ['s wrapped m',fmt(f.s_wrapped)], ['s 連續 m',fmt(f.s_unwrapped)], ['橫向 d m',fmt(f.d)], ['航向誤差 rad',fmt(f.e_psi)], ['模擬絕對時間',fmt(frame.sim_time_s) + ' s']]);
  el('candidates').replaceChildren();
  if (!frame.plan) {
    el('planStatus').textContent = '尚無候選'; el('planValues').replaceChildren();
  } else {
    const p = frame.plan;
    el('planStatus').textContent = p.status === 'completed' ? `中心點／折線取樣通過 ${p.sampled_valid_count} / 5` : '本次規劃失敗／跳過：' + (p.reason || p.status);
    values('planValues', [['規劃事件', '#' + p.plan_id], ['來源步號', p.source_control_step], ['來源相對時間',fmt(p.source_sim_time_s - data.origin_sim_time_s) + ' s'], ['候選年齡',fmt(frame.candidate_age_s) + ' s'], ['運算耗時',fmt(p.wall_time_s) + ' s']]);
    for (const path of p.paths) {
      const item = document.createElement('div'); item.className = 'candidate';
      const title = document.createElement('strong'); title.style.color = colors[path.candidate_id - 1];
      title.textContent = `候選 ${path.candidate_id} · ` + (path.sampled_valid ? '取樣通過' : '拒絕／跳過');
      item.append(title);
      const detail = document.createElement('div'); detail.className = path.reason ? 'reason' : 'muted';
      detail.textContent = path.reason || '終點 d：' + fmt(path.actual_target_d) + ' m';
      item.append(detail);
      if (!path.pixels.some(Boolean)) {
        const empty = document.createElement('div'); empty.className = 'muted'; empty.textContent = '無可繪製路徑'; item.append(empty);
      }
      el('candidates').append(item);
    }
  }
  if (el('follow').checked) centerCar(); updateStatus(); draw();
}
async function loadFrame(query, stop = true, epoch = null) {
  if (stop) pause();
  const token = ++requestSequence;
  if (pending) pending.abort(); pending = new AbortController();
  loading = true; updateStatus();
  el('error').textContent = '';
  try {
    const next = await api('/api/frame?' + query, pending.signal);
    if (token !== requestSequence || (epoch !== null && epoch !== playEpoch)) return false;
    apply(next); return true;
  } catch (error) {
    if (error.name !== 'AbortError' && token === requestSequence) {
      pause(); el('error').textContent = '無法跳轉：' + error.message;
    }
    return false;
  } finally {
    if (token === requestSequence) { loading = false; updateStatus(); }
  }
}
function schedule(epoch) {
  if (!playing || epoch !== playEpoch || !frame || frame.at_end) {
    if (frame && frame.at_end) pause(); return;
  }
  const next = cursor + 1;
  const due = anchorWall + (data.timeline[next] - anchorSim) * 1000 / Number(el('rate').value);
  timer = setTimeout(async () => {
    if (!playing || epoch !== playEpoch) return;
    if (await loadFrame('step=' + next, false, epoch)) schedule(epoch);
  }, Math.max(0, due - performance.now()));
}
function play() {
  if (!frame || frame.at_end || loading) return;
  playing = true; const epoch = ++playEpoch;
  anchorWall = performance.now(); anchorSim = data.timeline[cursor]; updateStatus(); schedule(epoch);
}
function jumpStep() {
  pause(); const raw = el('stepInput').value, value = Number(raw);
  if (!raw.trim() || !Number.isInteger(value) || value < 0 || value >= data.state_count) {
    el('error').textContent = `步號必須介於 0～${data.state_count - 1} 的整數`; return;
  }
  loadFrame('step=' + value);
}
function jumpTime() {
  pause(); const raw = el('timeInput').value, value = Number(raw);
  if (!raw.trim() || !Number.isFinite(value) || value < 0 || value > data.duration_s) {
    el('error').textContent = `相對時間必須介於 0～${fmt(data.duration_s)} 秒`; return;
  }
  loadFrame('time=' + encodeURIComponent(value));
}
el('play').onclick = () => playing ? pause() : play();
el('home').onclick = () => loadFrame('step=0');
el('previous').onclick = () => loadFrame('step=' + Math.max(0, cursor - 1));
el('next').onclick = () => loadFrame('step=' + Math.min(data.state_count - 1, cursor + 1));
el('timeline').oninput = () => loadFrame('step=' + el('timeline').value);
el('jumpStep').onclick = jumpStep; el('jumpTime').onclick = jumpTime;
el('stepInput').onfocus = pause; el('timeInput').onfocus = pause;
el('stepInput').onkeydown = event => { if (event.key === 'Enter') jumpStep(); };
el('timeInput').onkeydown = event => { if (event.key === 'Enter') jumpTime(); };
el('rate').onchange = () => { if (playing) { pause(); play(); } };
el('fit').onclick = fit;
el('focusCar').onclick = () => { centerCar(true); draw(); };
el('follow').onchange = () => { if (el('follow').checked) centerCar(true); draw(); };
['center','bounds','trail','paths','source'].forEach(id => el(id).onchange = draw);
canvas.addEventListener('wheel', event => {
  event.preventDefault(); el('follow').checked = false;
  const rect = canvas.getBoundingClientRect(), x = event.clientX - rect.left, y = event.clientY - rect.top;
  const next = Math.max(.05, Math.min(60, scale * Math.exp(-event.deltaY * .001)));
  ox = x - (x - ox) * next / scale; oy = y - (y - oy) * next / scale; scale = next; draw();
}, {passive:false});
canvas.addEventListener('pointerdown', event => { el('follow').checked = false; drag = {x:event.offsetX,y:event.offsetY,ox,oy}; canvas.setPointerCapture(event.pointerId); });
canvas.addEventListener('pointermove', event => { if (drag) { ox = drag.ox + event.offsetX - drag.x; oy = drag.oy + event.offsetY - drag.y; draw(); } });
canvas.addEventListener('pointerup', () => drag = null); canvas.addEventListener('pointercancel', () => drag = null);
window.addEventListener('resize', resize);
document.addEventListener('keydown', event => {
  if (!frame || ['INPUT','SELECT','BUTTON'].includes(event.target.tagName)) return;
  if (event.code === 'Space') { event.preventDefault(); playing ? pause() : play(); }
  if (event.key === 'ArrowLeft') { event.preventDefault(); el('previous').click(); }
  if (event.key === 'ArrowRight') { event.preventDefault(); el('next').click(); }
});
async function init() {
  data = await api('/api/session'); image = new Image(); image.src = '/api/image'; await image.decode();
  el('sessionName').textContent = 'ESP · ' + data.session_name + ' · ' + (data.recording_complete ? data.completed_full_laps + ' 圈' : '短預覽／未完成整圈');
  el('planningSettings').textContent = '前視 ' + fmt(data.lookahead_m, 1) + ' m · 候選每 ' + fmt(data.plan_period_s, 1) + ' s 更新 · 車況每 ' + fmt(data.control_dt_s, 1) + ' s 記錄';
  el('version').textContent = data.geometry_id;
  el('indexStatus').textContent = data.index_status === 'verified' ? '記錄索引核對通過' : '索引已由原始記錄重建；未修改 session';
  el('pathLegend').replaceChildren();
  colors.forEach((color,i) => { const span = document.createElement('span'); span.style.color = color; span.textContent = '● 候選 ' + (i + 1); el('pathLegend').append(span); });
  el('timeline').max = data.state_count - 1; el('stepInput').max = data.state_count - 1; el('timeInput').max = data.duration_s;
  ['home','play','next','jumpStep','jumpTime','timeline'].forEach(id => el(id).disabled = false);
  resize(); await loadFrame('step=0');
  if (data.generation_progress_enabled) {
    const poll = async () => {
      try {
        const p = await api('/api/progress');
        el('generationProgress').textContent = p.status === 'running' ?
          `40 m 完整圈離線生成中 · 已保存 ${p.plan_count} 組 · 最新控制步 ${p.control_step}` :
          p.status === 'complete' ? '40 m 完整圈資料已生成完成；目前仍顯示此預覽 session' :
          p.status === 'stopped' ? '完整圈生成已停止：' + (p.error || p.end_reason) : '完整圈生成正在啟動';
      } catch (error) { el('generationProgress').textContent = '暫時無法讀取生成進度'; }
    };
    await poll(); setInterval(poll, 10000);
  }
}
init().catch(error => { el('stateStatus').textContent = '無法載入 session'; el('error').textContent = error.message; });
