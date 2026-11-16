'use strict';

// Easing functions -- must mirror python/robot_kit/easing.py exactly so the
// on-screen curve preview matches what the robot actually does.
const EASING = {
  linear: (t) => t,
  easeInOutQuad: (t) => (t < 0.5 ? 2 * t * t : -1 + (4 - 2 * t) * t),
  easeOutElastic: (t) => {
    if (t === 0 || t === 1) return t;
    const p = 0.3;
    return Math.pow(2, -10 * t) * Math.sin(((t - p / 4) * (2 * Math.PI)) / p) + 1;
  },
  easeInOutSine: (t) => -(Math.cos(Math.PI * t) - 1) / 2,
  easeInOutCubic: (t) => (t < 0.5 ? 4 * t * t * t : (t - 1) * (2 * t - 2) ** 2 + 1),
  easeInOutPoly: (t) => (t < 0.5 ? 16 * t ** 5 : 1 - 16 * (1 - t) ** 5),
};

// Motor set + ranges mirror config/m_sarg.yaml / servo_driver_node.py's
// MOTOR_DEFAULTS. Purely UI-side bounds for the sliders -- the robot itself
// is the source of truth and re-clamps everything server-side.
const MOTORS = [
  { name: 'base_rotation', label: 'Base Rotation', range: [-120, 120] },
  { name: 'head_pitch', label: 'Head Pitch', range: [-40, 40] },
  { name: 'head_yaw', label: 'Head Yaw', range: [-90, 90] },
  { name: 'arm_left', label: 'Arm Left', range: [-45, 135] },
  { name: 'arm_right', label: 'Arm Right', range: [-45, 135] },
];

const TRACK_H = 70;
const TRACK_MARGIN = 10;

// Must mirror face_node.py's VALID_EXPRESSIONS -- anything else is logged
// and ignored by the face, so keep this list in sync with that module.
const EXPRESSIONS = [
  { name: 'neutral', label: 'Neutral', color: '#9098a8' },
  { name: 'happy', label: 'Happy', color: '#4fd67a' },
  { name: 'sad', label: 'Sad', color: '#4f8cff' },
  { name: 'surprised', label: 'Surprised', color: '#ffb44f' },
  { name: 'blink', label: 'Blink', color: '#c084fc' },
];

const state = {
  totalDuration: 3.0,
  pxPerSecond: 150,
  motors: {},
  expressions: [], // [{ time, duration, name }], face expression blocks
};

// { kind: 'motor', motor, index } into state.motors[motor], or
// { kind: 'expression', block } pointing at an object in state.expressions.
let selected = null;
let currentMode = 'timeline'; // 'timeline' | 'live' -- see setMode()

// Live-control (jog) state, separate from the timeline's `state.motors`.
// `pose` holds the last angle sent/known for each motor; `initialized`
// guards the one-time GET /pose sync that happens on first switching to
// the Live Control tab.
const live = {
  pose: {},
  initialized: false,
};
const liveThrottle = {}; // motor name -> performance.now() of last POST /pose
const LIVE_SEND_INTERVAL_MS = 60; // ~16Hz cap while dragging a slider

// ---------------------------------------------------------------------------
// Script <-> track-point conversion
//
// robot_kit keyframes describe a SEGMENT: starting at time `at`, ramping over
// `duration` seconds to reach `pose`'s angle (see Timeline/_ActiveTimeline in
// python/robot_kit/timeline.py + animator.py). The editor instead models each
// track as arrival-time points (the angle the motor should reach BY time t),
// which is the more natural authoring mental model. buildScript()/parseScript()
// are exact inverses that translate between the two.
// ---------------------------------------------------------------------------

function buildScript() {
  const script = [];
  for (const m of MOTORS) {
    const pts = [...(state.motors[m.name] || [])].sort((a, b) => a.time - b.time);
    if (pts.length === 0) continue;
    // Anchor: pin the first point's value at its own time (instantaneous).
    script.push({ at: round2(pts[0].time), pose: { [m.name]: pts[0].angle }, duration: 0 });
    for (let i = 1; i < pts.length; i++) {
      const prev = pts[i - 1];
      const cur = pts[i];
      script.push({
        at: round2(prev.time),
        pose: { [m.name]: cur.angle },
        duration: round2(Math.max(0, cur.time - prev.time)),
        easing: cur.easing || 'easeInOutSine',
      });
    }
  }
  return script;
}

function parseScript(script) {
  const byMotor = {};
  MOTORS.forEach((m) => {
    byMotor[m.name] = [];
  });
  for (const kf of script) {
    if (!kf || typeof kf.pose !== 'object') continue;
    const at = Number(kf.at) || 0;
    let duration = Number(kf.duration || 0);
    if (kf.end_time !== undefined) duration = Number(kf.end_time) - at;
    for (const [motor, angle] of Object.entries(kf.pose)) {
      if (!(motor in byMotor)) continue;
      const easing = typeof kf.easing === 'object' ? kf.easing[motor] : kf.easing;
      const time = duration > 0 ? at + duration : at;
      byMotor[motor].push({
        time: round2(time),
        angle: Number(angle),
        easing: easing || 'easeInOutSine',
      });
    }
  }
  for (const name of Object.keys(byMotor)) {
    byMotor[name].sort((a, b) => a.time - b.time);
  }
  return byMotor;
}

function round2(n) {
  return Math.round(n * 100) / 100;
}
function round1(n) {
  return Math.round(n * 10) / 10;
}

// ---------------------------------------------------------------------------
// Expression block <-> event conversion
//
// state.expressions holds authoring blocks { time, duration, name }, which
// attachExpressionHandlers() keeps non-overlapping by construction. Playback
// (preview + /expressions) only cares about the *events* implied by those
// blocks: fire `name` at `time`, and -- if nothing else is already scheduled
// to start right where this block ends -- revert to 'neutral' at the end of
// the block. buildExpressionEvents() derives that flattened, sorted event
// list without mutating state.expressions.
// ---------------------------------------------------------------------------

function buildExpressionEvents() {
  const blocks = [...state.expressions].sort((a, b) => a.time - b.time);
  const events = [];
  blocks.forEach((b, i) => {
    events.push({ at: round2(b.time), name: b.name });
    const end = round2(b.time + b.duration);
    const next = blocks[i + 1];
    const touchesNext = next && Math.abs(next.time - end) < 1e-6;
    if (!touchesNext && end < state.totalDuration - 1e-6) {
      events.push({ at: end, name: 'neutral' });
    }
  });
  return events;
}

// The earliest a block may start (right after the previous block ends) and
// the latest its end may reach (right where the next block starts, or the
// end of the timeline) -- used to keep blocks from overlapping while
// dragging/resizing/editing.
function neighborBounds(block) {
  const list = [...state.expressions].sort((a, b) => a.time - b.time);
  const idx = list.indexOf(block);
  const prev = list[idx - 1];
  const next = list[idx + 1];
  return {
    prevEnd: prev ? round2(prev.time + prev.duration) : 0,
    nextStart: next ? next.time : state.totalDuration,
  };
}

// ---------------------------------------------------------------------------
// Geometry helpers
// ---------------------------------------------------------------------------

function timeToPx(t) {
  return t * state.pxPerSecond;
}
function pxToTime(px) {
  return px / state.pxPerSecond;
}
function angleToY(angle, range) {
  const [lo, hi] = range;
  const frac = (angle - lo) / (hi - lo);
  return TRACK_H - TRACK_MARGIN - frac * (TRACK_H - 2 * TRACK_MARGIN);
}
function yToAngle(y, range) {
  const [lo, hi] = range;
  const frac = 1 - (y - TRACK_MARGIN) / (TRACK_H - 2 * TRACK_MARGIN);
  return Math.max(lo, Math.min(hi, lo + frac * (hi - lo)));
}
function laneWidth() {
  return Math.max(400, timeToPx(state.totalDuration));
}

function sampleSegmentPath(prev, cur, range) {
  const fn = EASING[cur.easing] || EASING.easeInOutSine;
  const steps = 20;
  const pts = [];
  for (let i = 0; i <= steps; i++) {
    const rel = i / steps;
    const angle = prev.angle + (cur.angle - prev.angle) * fn(rel);
    const time = prev.time + (cur.time - prev.time) * rel;
    pts.push([timeToPx(time), angleToY(angle, range)]);
  }
  return pts;
}

// ---------------------------------------------------------------------------
// Rendering
// ---------------------------------------------------------------------------

function renderAll() {
  renderRuler();
  renderTracks();
  renderInspector();
}

function renderRuler() {
  const ruler = document.getElementById('ruler');
  ruler.innerHTML = '';
  ruler.style.width = laneWidth() + 'px';
  const step = state.totalDuration > 10 ? 1 : 0.5;
  for (let t = 0; t <= state.totalDuration + 1e-9; t += step) {
    const tick = document.createElement('div');
    tick.className = 'ruler-tick';
    tick.style.left = timeToPx(t) + 'px';
    tick.textContent = t.toFixed(step < 1 ? 1 : 0) + 's';
    ruler.appendChild(tick);
  }
}

function renderTracks() {
  const container = document.getElementById('tracks');
  container.innerHTML = '';

  for (const m of MOTORS) {
    const track = document.createElement('div');
    track.className = 'track';

    const label = document.createElement('div');
    label.className = 'track-label';
    label.innerHTML = `<strong>${m.label}</strong><span>${m.range[0]}° … ${m.range[1]}°</span>`;
    track.appendChild(label);

    const lane = document.createElement('div');
    lane.className = 'track-lane';
    lane.style.width = laneWidth() + 'px';

    const zero = document.createElement('div');
    zero.className = 'zero-line';
    zero.style.top = angleToY(0, m.range) + 'px';
    lane.appendChild(zero);

    const pts = [...(state.motors[m.name] || [])].sort((a, b) => a.time - b.time);

    if (pts.length > 1) {
      const svg = document.createElementNS('http://www.w3.org/2000/svg', 'svg');
      svg.setAttribute('class', 'kf-path');
      svg.setAttribute('width', String(laneWidth()));
      svg.setAttribute('height', String(TRACK_H));
      for (let i = 1; i < pts.length; i++) {
        const seg = sampleSegmentPath(pts[i - 1], pts[i], m.range);
        const poly = document.createElementNS('http://www.w3.org/2000/svg', 'polyline');
        poly.setAttribute('points', seg.map((p) => p.join(',')).join(' '));
        poly.setAttribute('fill', 'none');
        poly.setAttribute('stroke', '#4f8cff');
        poly.setAttribute('stroke-width', '2');
        poly.setAttribute('opacity', '0.6');
        svg.appendChild(poly);
      }
      lane.appendChild(svg);
    }

    pts.forEach((pt, idx) => {
      const el = document.createElement('div');
      el.className = 'kf-point';
      if (selected && selected.kind === 'motor' && selected.motor === m.name && selected.index === idx) {
        el.classList.add('selected');
      }
      el.style.left = timeToPx(pt.time) + 'px';
      el.style.top = angleToY(pt.angle, m.range) + 'px';
      attachPointHandlers(el, m, pt);
      lane.appendChild(el);
    });

    lane.addEventListener('click', (ev) => {
      if (ev.target !== lane) return; // ignore clicks that landed on a point/path
      const rect = lane.getBoundingClientRect();
      const time = Math.max(0, Math.min(state.totalDuration, pxToTime(ev.clientX - rect.left)));
      const angle = yToAngle(ev.clientY - rect.top, m.range);
      const list = state.motors[m.name];
      const newPoint = { time: round2(time), angle: round1(angle), easing: 'easeInOutSine' };
      list.push(newPoint);
      list.sort((a, b) => a.time - b.time);
      selected = { kind: 'motor', motor: m.name, index: list.indexOf(newPoint) };
      renderAll();
    });

    track.appendChild(lane);
    container.appendChild(track);
  }

  renderExpressionTrack(container);
}

function renderExpressionTrack(container) {
  const track = document.createElement('div');
  track.className = 'track';

  const label = document.createElement('div');
  label.className = 'track-label';
  label.innerHTML = '<strong>Face Expression</strong><span>tap to add</span>';
  track.appendChild(label);

  const lane = document.createElement('div');
  lane.className = 'track-lane expr-lane';
  lane.style.width = laneWidth() + 'px';

  const blocks = [...state.expressions].sort((a, b) => a.time - b.time);
  blocks.forEach((block) => {
    const el = document.createElement('div');
    el.className = 'expr-block';
    if (selected && selected.kind === 'expression' && selected.block === block) {
      el.classList.add('selected');
    }
    const def = EXPRESSIONS.find((e) => e.name === block.name) || EXPRESSIONS[0];
    el.style.left = timeToPx(block.time) + 'px';
    el.style.width = Math.max(4, timeToPx(block.duration)) + 'px';
    el.style.background = def.color;
    el.textContent = def.label;

    const handle = document.createElement('div');
    handle.className = 'expr-resize-handle';
    el.appendChild(handle);

    attachExpressionHandlers(el, handle, block);
    lane.appendChild(el);
  });

  lane.addEventListener('click', (ev) => {
    if (ev.target !== lane) return; // ignore clicks that landed on a block/handle
    const rect = lane.getBoundingClientRect();
    const time = Math.max(0, Math.min(state.totalDuration, pxToTime(ev.clientX - rect.left)));
    const sorted = [...state.expressions].sort((a, b) => a.time - b.time);
    const next = sorted.find((b) => b.time > time + 1e-6);
    const maxDuration = (next ? next.time : state.totalDuration) - time;
    if (maxDuration < 0.1) return; // no room for a block here
    const block = { time: round2(time), duration: round2(Math.min(1.0, maxDuration)), name: 'happy' };
    state.expressions.push(block);
    selected = { kind: 'expression', block };
    renderAll();
  });

  track.appendChild(lane);
  container.appendChild(track);
}

function attachExpressionHandlers(el, handle, block) {
  el.addEventListener('pointerdown', (ev) => {
    ev.preventDefault();
    ev.stopPropagation();
    let moved = false;
    const pointerId = ev.pointerId;
    el.classList.add('dragging');
    const lane = el.parentElement;
    const rect = lane.getBoundingClientRect();
    const grabOffsetPx = ev.clientX - rect.left - timeToPx(block.time);

    function onMove(mv) {
      if (mv.pointerId !== pointerId) return;
      moved = true;
      const { prevEnd, nextStart } = neighborBounds(block);
      const maxTime = Math.max(prevEnd, nextStart - block.duration);
      const rawTime = pxToTime(mv.clientX - rect.left - grabOffsetPx);
      block.time = round2(Math.max(prevEnd, Math.min(maxTime, rawTime)));
      selected = { kind: 'expression', block };
      renderAll();
    }
    function onUp(up) {
      if (up.pointerId !== pointerId) return;
      document.removeEventListener('pointermove', onMove);
      document.removeEventListener('pointerup', onUp);
      document.removeEventListener('pointercancel', onUp);
      if (!moved) {
        selected = { kind: 'expression', block };
        renderAll();
      }
    }
    document.addEventListener('pointermove', onMove);
    document.addEventListener('pointerup', onUp);
    document.addEventListener('pointercancel', onUp);
  });

  handle.addEventListener('pointerdown', (ev) => {
    ev.preventDefault();
    ev.stopPropagation();
    const pointerId = ev.pointerId;
    el.classList.add('dragging');
    const lane = el.parentElement;
    const rect = lane.getBoundingClientRect();

    function onMove(mv) {
      if (mv.pointerId !== pointerId) return;
      const { nextStart } = neighborBounds(block);
      const rawEnd = pxToTime(mv.clientX - rect.left);
      block.duration = round2(Math.max(0.05, Math.min(nextStart, rawEnd) - block.time));
      selected = { kind: 'expression', block };
      renderAll();
    }
    function onUp(up) {
      if (up.pointerId !== pointerId) return;
      document.removeEventListener('pointermove', onMove);
      document.removeEventListener('pointerup', onUp);
      document.removeEventListener('pointercancel', onUp);
    }
    document.addEventListener('pointermove', onMove);
    document.addEventListener('pointerup', onUp);
    document.addEventListener('pointercancel', onUp);
  });
}

function attachPointHandlers(el, motorDef, point) {
  // Pointer Events unify mouse/touch/pen (a plain tap on touch still fires
  // pointerdown+pointerup with no move), which is what makes keyframes
  // draggable on a phone -- the old mousedown-only handlers never fired
  // on touch at all. Move/up listeners go on `document` rather than `el`
  // (mirroring the previous mouse implementation) because renderAll()
  // tears down and rebuilds every point's DOM node on each edit, which
  // would otherwise orphan listeners attached to `el` mid-drag.
  el.addEventListener('pointerdown', (ev) => {
    ev.preventDefault();
    ev.stopPropagation();
    let moved = false;
    const pointerId = ev.pointerId;
    el.classList.add('dragging');
    const lane = el.parentElement;
    const rect = lane.getBoundingClientRect();

    function onMove(mv) {
      if (mv.pointerId !== pointerId) return;
      moved = true;
      const time = Math.max(0, Math.min(state.totalDuration, pxToTime(mv.clientX - rect.left)));
      const angle = yToAngle(mv.clientY - rect.top, motorDef.range);
      point.time = round2(time);
      point.angle = round1(angle);
      const list = state.motors[motorDef.name];
      list.sort((a, b) => a.time - b.time);
      selected = { kind: 'motor', motor: motorDef.name, index: list.indexOf(point) };
      renderAll();
    }
    function onUp(up) {
      if (up.pointerId !== pointerId) return;
      document.removeEventListener('pointermove', onMove);
      document.removeEventListener('pointerup', onUp);
      document.removeEventListener('pointercancel', onUp);
      if (!moved) {
        selected = { kind: 'motor', motor: motorDef.name, index: state.motors[motorDef.name].indexOf(point) };
        renderAll();
      }
    }
    document.addEventListener('pointermove', onMove);
    document.addEventListener('pointerup', onUp);
    document.addEventListener('pointercancel', onUp);
  });
}

function renderInspector() {
  const aside = document.getElementById('inspector');
  if (!selected || currentMode !== 'timeline') {
    aside.hidden = true;
    return;
  }

  if (selected.kind === 'expression') {
    const block = selected.block;
    if (!state.expressions.includes(block)) {
      selected = null;
      aside.hidden = true;
      return;
    }
    aside.hidden = false;
    document.getElementById('inspectorMotorFields').hidden = true;
    document.getElementById('inspectorExprFields').hidden = false;
    document.getElementById('insExprName').value = block.name;
    document.getElementById('insExprTime').value = block.time.toFixed(2);
    document.getElementById('insExprDuration').value = block.duration.toFixed(2);
    return;
  }

  const list = state.motors[selected.motor] || [];
  const pt = list[selected.index];
  if (!pt) {
    selected = null;
    aside.hidden = true;
    return;
  }
  aside.hidden = false;
  document.getElementById('inspectorMotorFields').hidden = false;
  document.getElementById('inspectorExprFields').hidden = true;
  document.getElementById('insMotor').textContent = selected.motor;
  document.getElementById('insTime').value = pt.time.toFixed(2);
  document.getElementById('insAngle').value = pt.angle;
  document.getElementById('insEasing').value = pt.easing || 'easeInOutSine';
}

function deleteSelected() {
  if (!selected) return;
  if (selected.kind === 'expression') {
    const idx = state.expressions.indexOf(selected.block);
    if (idx >= 0) state.expressions.splice(idx, 1);
  } else {
    state.motors[selected.motor].splice(selected.index, 1);
  }
  selected = null;
  renderAll();
}

// ---------------------------------------------------------------------------
// Robot connectivity
// ---------------------------------------------------------------------------

function robotUrl() {
  return document.getElementById('robotUrl').value.trim().replace(/\/+$/, '');
}

function setStatus(msg, kind) {
  const el = document.getElementById('statusLine');
  el.textContent = msg;
  el.className = 'status-line' + (kind ? ' ' + kind : '');
}

async function testConnection() {
  const url = robotUrl();
  const dot = document.getElementById('connDot');
  const label = document.getElementById('connLabel');
  if (!url) {
    setStatus('Enter a robot URL first.', 'err');
    return;
  }
  dot.className = 'dot dot-unknown';
  label.textContent = 'testing…';
  try {
    const res = await fetch(url + '/health');
    if (!res.ok) throw new Error('HTTP ' + res.status);
    const data = await res.json();
    dot.className = 'dot ' + (data.servo_driver_connected ? 'dot-ok' : 'dot-bad');
    label.textContent = data.servo_driver_connected
      ? 'connected (servo driver up)'
      : 'rk_server up, servo driver unreachable';
    setStatus('Connection OK.', 'ok');
    fetchStaticBehaviors();
  } catch (err) {
    dot.className = 'dot dot-bad';
    label.textContent = 'unreachable';
    setStatus('Connection failed: ' + err.message, 'err');
  }
}

async function fetchStaticBehaviors() {
  const url = robotUrl();
  const select = document.getElementById('staticBehaviorSelect');
  if (!url) return;
  try {
    const res = await fetch(url + '/static_behaviors');
    if (!res.ok) throw new Error('HTTP ' + res.status);
    const data = await res.json();
    const names = Array.isArray(data.static_behaviors) ? data.static_behaviors : [];
    select.innerHTML = '';
    if (names.length === 0) {
      select.innerHTML = '<option value="">(none available)</option>';
      return;
    }
    for (const name of names) {
      const opt = document.createElement('option');
      opt.value = name;
      opt.textContent = name;
      select.appendChild(opt);
    }
  } catch (err) {
    select.innerHTML = '<option value="">(failed to load)</option>';
  }
}

async function playStaticBehavior() {
  const url = robotUrl();
  if (!url) {
    setStatus('Enter a robot URL first.', 'err');
    return;
  }
  const name = document.getElementById('staticBehaviorSelect').value;
  if (!name) {
    setStatus('Choose a static behavior first.', 'err');
    return;
  }
  const speed_scale = parseFloat(document.getElementById('speedScale').value) || 1.0;
  setStatus('Playing ' + name + '…');
  try {
    const res = await fetch(url + '/static_behavior', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ name, speed_scale }),
    });
    const data = await res.json().catch(() => ({}));
    if (res.ok) {
      setStatus('Playing ' + name + '.', 'ok');
    } else {
      setStatus('Robot rejected the behavior: ' + (data.error || res.status), 'err');
    }
  } catch (err) {
    setStatus('Play failed: ' + err.message, 'err');
  }
}

async function sendToRobot() {
  const url = robotUrl();
  if (!url) {
    setStatus('Enter a robot URL first.', 'err');
    return;
  }
  const script = buildScript();
  const events = buildExpressionEvents();
  if (script.length === 0 && events.length === 0) {
    setStatus('Nothing to send — add at least one keyframe or expression.', 'err');
    return;
  }
  const speed_scale = parseFloat(document.getElementById('speedScale').value) || 1.0;
  setStatus('Sending…');
  try {
    const calls = [];
    if (script.length > 0) {
      calls.push(
        fetch(url + '/animate', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ script, speed_scale }),
        }).then((res) => res.json().catch(() => ({})).then((data) => ({ res, data, kind: 'animation' })))
      );
    }
    if (events.length > 0) {
      calls.push(
        fetch(url + '/expressions', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ events, speed_scale }),
        }).then((res) => res.json().catch(() => ({})).then((data) => ({ res, data, kind: 'expressions' })))
      );
    }
    const results = await Promise.all(calls);
    const failed = results.find((r) => !r.res.ok);
    if (failed) {
      setStatus(`Robot rejected the ${failed.kind}: ` + (failed.data.error || failed.res.status), 'err');
      return;
    }
    setStatus('Sent.', 'ok');
  } catch (err) {
    setStatus('Send failed: ' + err.message, 'err');
  }
}

// ---------------------------------------------------------------------------
// Export / import / preview
// ---------------------------------------------------------------------------

function behaviorPayload() {
  const name = document.getElementById('animName').value.trim();
  const script = buildScript();
  const expressions = [...state.expressions]
    .sort((a, b) => a.time - b.time)
    .map((b) => ({ at: round2(b.time), duration: round2(b.duration), name: b.name }));
  return { name: name || null, script, expressions };
}

function exportJSON() {
  const payload = behaviorPayload();
  const blob = new Blob([JSON.stringify(payload, null, 2)], { type: 'application/json' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = (payload.name || 'm_sarg_animation') + '.json';
  a.click();
  URL.revokeObjectURL(a.href);
}

function importJSON(file) {
  const reader = new FileReader();
  reader.onload = () => {
    try {
      const parsed = JSON.parse(reader.result);
      // Accept both the named { name, script } format this editor exports
      // and a bare keyframe array (e.g. hand-written or from server-rk.py).
      const script = Array.isArray(parsed) ? parsed : parsed.script;
      if (!Array.isArray(script)) {
        throw new Error("file must contain a JSON array, or an object with a 'script' array");
      }
      const rawExpressions = (!Array.isArray(parsed) && parsed.expressions) || [];
      document.getElementById('animName').value =
        (!Array.isArray(parsed) && parsed.name) || '';
      state.motors = parseScript(script);
      state.expressions = Array.isArray(rawExpressions)
        ? rawExpressions
            .filter((e) => e && typeof e.name === 'string')
            .map((e) => ({
              time: round2(Number(e.at ?? e.time) || 0),
              duration: round2(Math.max(0.05, Number(e.duration) || 1)),
              name: e.name,
            }))
            .sort((a, b) => a.time - b.time)
        : [];
      selected = null;
      renderAll();
      setStatus('Imported ' + file.name + '.', 'ok');
    } catch (err) {
      setStatus('Import failed: ' + err.message, 'err');
    }
  };
  reader.readAsText(file);
}

function trackLaneLeftOffset() {
  // #playhead is position:absolute inside main#timeline, so its `left` is
  // measured from timeline's padding box -- it does NOT get pushed right by
  // timeline's own left padding the way normal-flow elements (the lanes) do.
  // Measure the real gap instead of hardcoding it, so this stays correct if
  // the track-label width or timeline padding ever changes.
  const lane = document.querySelector('.track-lane');
  const container = document.getElementById('timeline');
  if (!lane || !container) return 150;
  return lane.getBoundingClientRect().left - container.getBoundingClientRect().left;
}

function preview() {
  const playhead = document.getElementById('playhead');
  const speed = parseFloat(document.getElementById('speedScale').value) || 1.0;
  const duration = state.totalDuration / speed;
  const width = laneWidth();
  const offset = trackLaneLeftOffset();
  playhead.hidden = false;
  const start = performance.now();
  function frame(now) {
    const frac = Math.min(1, (now - start) / 1000 / duration);
    playhead.style.left = offset + frac * width + 'px';
    updateActiveExpressionHighlight(frac * state.totalDuration);
    if (frac < 1) {
      requestAnimationFrame(frame);
    } else {
      setTimeout(() => {
        playhead.hidden = true;
        updateActiveExpressionHighlight(null);
      }, 200);
    }
  }
  requestAnimationFrame(frame);
}

// Highlights whichever expr-block is active at time `t` (seconds), or clears
// the highlight entirely when `t` is null. Relies on the .expr-block elements
// being appended in the same time-sorted order as state.expressions, so
// positional matching (rather than re-querying per block) is enough.
function updateActiveExpressionHighlight(t) {
  const lane = document.querySelector('.expr-lane');
  if (!lane) return;
  const blocks = [...state.expressions].sort((a, b) => a.time - b.time);
  const elements = lane.querySelectorAll('.expr-block');
  elements.forEach((el, i) => {
    const b = blocks[i];
    const isActive = t != null && b && t >= b.time - 1e-6 && t < b.time + b.duration - 1e-6;
    el.classList.toggle('active', !!isActive);
  });
}

// ---------------------------------------------------------------------------
// Mode switching (Timeline <-> Live Control)
// ---------------------------------------------------------------------------

function setMode(mode) {
  currentMode = mode;
  document.querySelectorAll('.mode-tab').forEach((btn) => {
    btn.classList.toggle('active', btn.dataset.mode === mode);
  });
  document.getElementById('timeline').hidden = mode !== 'timeline';
  document.getElementById('liveControl').hidden = mode !== 'live';
  renderInspector();

  if (mode === 'live' && !live.initialized) {
    live.initialized = true;
    syncPoseFromRobot();
  }
}

// ---------------------------------------------------------------------------
// Live control -- jog the real robot's pose directly via POST /pose,
// independent of the timeline's keyframe script.
// ---------------------------------------------------------------------------

function buildLiveMotors() {
  const container = document.getElementById('liveMotors');
  container.innerHTML = '';
  MOTORS.forEach((m) => {
    if (!(m.name in live.pose)) live.pose[m.name] = 0;

    const card = document.createElement('div');
    card.className = 'live-motor';

    const head = document.createElement('div');
    head.className = 'live-motor-head';
    const label = document.createElement('strong');
    label.textContent = m.label;
    const value = document.createElement('span');
    value.className = 'live-value';
    value.id = `liveVal-${m.name}`;
    head.appendChild(label);
    head.appendChild(value);
    card.appendChild(head);

    const slider = document.createElement('input');
    slider.type = 'range';
    slider.className = 'live-slider';
    slider.min = String(m.range[0]);
    slider.max = String(m.range[1]);
    slider.step = '0.5';
    slider.value = String(live.pose[m.name]);
    slider.id = `liveSlider-${m.name}`;
    slider.addEventListener('input', () => {
      const angle = parseFloat(slider.value);
      live.pose[m.name] = angle;
      updateLiveValueLabel(m.name, angle);
      maybeSendLive(m.name, angle, false);
    });
    slider.addEventListener('change', () => {
      const angle = parseFloat(slider.value);
      live.pose[m.name] = angle;
      maybeSendLive(m.name, angle, true); // guarantee the released value lands
    });
    card.appendChild(slider);

    container.appendChild(card);
  });
  updateAllLiveValueLabels();
}

function updateLiveValueLabel(motorName, angle) {
  const el = document.getElementById(`liveVal-${motorName}`);
  if (el) el.textContent = angle.toFixed(1) + '°';
}
function updateAllLiveValueLabels() {
  MOTORS.forEach((m) => updateLiveValueLabel(m.name, live.pose[m.name] ?? 0));
}

// Push slider positions/labels to match live.pose without rebuilding the
// slider DOM -- rebuilding mid-drag (e.g. from renderLiveMotors ->
// buildLiveMotors) would drop the pointer's active drag on some browsers.
function renderLiveMotors() {
  MOTORS.forEach((m) => {
    const slider = document.getElementById(`liveSlider-${m.name}`);
    if (slider) slider.value = String(live.pose[m.name] ?? 0);
  });
  updateAllLiveValueLabels();
}

function maybeSendLive(motorName, angle, force) {
  if (!document.getElementById('liveSendToggle').checked) return;
  const now = performance.now();
  const last = liveThrottle[motorName] || 0;
  if (!force && now - last < LIVE_SEND_INTERVAL_MS) return;
  liveThrottle[motorName] = now;
  sendPose({ [motorName]: angle });
}

async function sendPose(poseObj) {
  const url = robotUrl();
  if (!url) return;
  try {
    const res = await fetch(url + '/pose', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ pose: poseObj }),
    });
    if (!res.ok) {
      const data = await res.json().catch(() => ({}));
      setStatus('Live send rejected: ' + (data.error || res.status), 'err');
    }
  } catch (err) {
    setStatus('Live send failed: ' + err.message, 'err');
  }
}

async function syncPoseFromRobot() {
  const url = robotUrl();
  if (!url) {
    setStatus('Enter a robot URL first.', 'err');
    return;
  }
  setStatus('Reading current pose from robot…');
  try {
    const res = await fetch(url + '/pose');
    const data = await res.json().catch(() => ({}));
    if (!res.ok) throw new Error(data.error || ('HTTP ' + res.status));
    MOTORS.forEach((m) => {
      if (typeof data.pose[m.name] === 'number') {
        live.pose[m.name] = data.pose[m.name];
      }
    });
    renderLiveMotors();
    setStatus('Synced live sliders to the robot’s current pose.', 'ok');
  } catch (err) {
    setStatus('Sync failed: ' + err.message, 'err');
  }
}

function centerAllLive() {
  MOTORS.forEach((m) => {
    live.pose[m.name] = 0;
  });
  renderLiveMotors();
  sendPose(Object.fromEntries(MOTORS.map((m) => [m.name, 0])));
  setStatus('Centered all motors.', 'ok');
}

function captureLiveToTimeline() {
  const t = Math.max(0, parseFloat(document.getElementById('captureTime').value) || 0);
  MOTORS.forEach((m) => {
    const angle = live.pose[m.name];
    if (typeof angle !== 'number') return;
    const list = state.motors[m.name];
    const existing = list.find((p) => Math.abs(p.time - t) < 1e-6);
    if (existing) {
      existing.angle = round1(angle);
    } else {
      list.push({ time: round2(t), angle: round1(angle), easing: 'easeInOutSine' });
      list.sort((a, b) => a.time - b.time);
    }
  });
  selected = null;
  setStatus(`Captured live pose as keyframes at t=${t.toFixed(2)}s.`, 'ok');
  setMode('timeline');
  renderAll();
}

// ---------------------------------------------------------------------------
// Wiring
// ---------------------------------------------------------------------------

function init() {
  MOTORS.forEach((m) => {
    state.motors[m.name] = [{ time: 0, angle: 0, easing: 'easeInOutSine' }];
  });

  const robotUrlInput = document.getElementById('robotUrl');
  const embedded = window.parent !== window && new URLSearchParams(location.search).get('embed') === 'bloom';
  if (embedded) {
    robotUrlInput.value = 'http://127.0.0.1:5500';
    const save = document.getElementById('btnSaveBloom');
    window.addEventListener('message', (event) => {
      if (event.origin !== location.origin || event.source !== window.parent) return;
      if (event.data?.type === 'bloom:animator-config') save.hidden = event.data.canSave !== true;
      if (event.data?.type === 'bloom:animator-saved') {
        save.disabled = false;
        setStatus(event.data.message, event.data.ok ? 'ok' : 'err');
      }
    });
    save.addEventListener('click', () => {
      const payload = behaviorPayload();
      if (!payload.name) return setStatus('Enter a behavior name before saving.', 'err');
      save.disabled = true;
      window.parent.postMessage({ type: 'bloom:save-behavior', payload }, location.origin);
    });
  } else if (location.protocol.startsWith('http')) {
    robotUrlInput.value = location.origin;
    fetchStaticBehaviors();
  }

  document.getElementById('totalDuration').addEventListener('change', (e) => {
    const v = parseFloat(e.target.value);
    state.totalDuration = Number.isNaN(v) ? 3 : Math.max(0.5, v);
    renderAll();
  });

  document.querySelectorAll('.mode-tab').forEach((btn) => {
    btn.addEventListener('click', () => setMode(btn.dataset.mode));
  });
  buildLiveMotors();
  document.getElementById('btnSyncPose').addEventListener('click', syncPoseFromRobot);
  document.getElementById('btnCenterAll').addEventListener('click', centerAllLive);
  document.getElementById('btnCapture').addEventListener('click', captureLiveToTimeline);

  document.getElementById('btnConnect').addEventListener('click', testConnection);
  document.getElementById('btnSend').addEventListener('click', sendToRobot);
  document.getElementById('btnPreview').addEventListener('click', preview);
  document.getElementById('btnPlayStatic').addEventListener('click', playStaticBehavior);
  document.getElementById('btnExport').addEventListener('click', exportJSON);
  document.getElementById('btnImport').addEventListener('click', () => {
    document.getElementById('importFile').click();
  });
  document.getElementById('importFile').addEventListener('change', (e) => {
    if (e.target.files[0]) importJSON(e.target.files[0]);
    e.target.value = '';
  });
  document.getElementById('btnClear').addEventListener('click', () => {
    if (!confirm('Clear all keyframes and expressions?')) return;
    MOTORS.forEach((m) => {
      state.motors[m.name] = [];
    });
    state.expressions = [];
    selected = null;
    renderAll();
  });

  document.getElementById('insTime').addEventListener('change', (e) => {
    if (!selected) return;
    const list = state.motors[selected.motor];
    const pt = list[selected.index];
    pt.time = Math.max(0, Math.min(state.totalDuration, parseFloat(e.target.value) || 0));
    list.sort((a, b) => a.time - b.time);
    selected.index = list.indexOf(pt);
    renderAll();
  });
  document.getElementById('insAngle').addEventListener('change', (e) => {
    if (!selected) return;
    const m = MOTORS.find((mm) => mm.name === selected.motor);
    const list = state.motors[selected.motor];
    const pt = list[selected.index];
    const val = parseFloat(e.target.value);
    pt.angle = Math.max(m.range[0], Math.min(m.range[1], Number.isNaN(val) ? 0 : val));
    renderAll();
  });
  document.getElementById('insEasing').addEventListener('change', (e) => {
    if (!selected || selected.kind !== 'motor') return;
    state.motors[selected.motor][selected.index].easing = e.target.value;
    renderAll();
  });
  document.getElementById('insExprName').addEventListener('change', (e) => {
    if (!selected || selected.kind !== 'expression') return;
    selected.block.name = e.target.value;
    renderAll();
  });
  document.getElementById('insExprTime').addEventListener('change', (e) => {
    if (!selected || selected.kind !== 'expression') return;
    const block = selected.block;
    const { prevEnd, nextStart } = neighborBounds(block);
    const maxTime = Math.max(prevEnd, nextStart - block.duration);
    const val = parseFloat(e.target.value);
    block.time = round2(Math.max(prevEnd, Math.min(maxTime, Number.isNaN(val) ? 0 : val)));
    renderAll();
  });
  document.getElementById('insExprDuration').addEventListener('change', (e) => {
    if (!selected || selected.kind !== 'expression') return;
    const block = selected.block;
    const { nextStart } = neighborBounds(block);
    const val = parseFloat(e.target.value);
    block.duration = round2(Math.max(0.05, Math.min(nextStart - block.time, Number.isNaN(val) ? 1 : val)));
    renderAll();
  });
  document.getElementById('btnDeletePoint').addEventListener('click', () => {
    if (!selected) return;
    deleteSelected();
  });

  document.addEventListener('keydown', (e) => {
    if ((e.key === 'Delete' || e.key === 'Backspace') && selected) {
      const tag = document.activeElement.tagName;
      if (tag === 'INPUT' || tag === 'SELECT' || tag === 'TEXTAREA') return;
      deleteSelected();
    }
  });

  renderAll();
}

document.addEventListener('DOMContentLoaded', init);
