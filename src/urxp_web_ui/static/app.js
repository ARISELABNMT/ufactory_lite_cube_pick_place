const $ = (sel) => document.querySelector(sel);

async function postJSON(url, body) {
  const res = await fetch(url, {
    method: 'POST',
    headers: body ? { 'Content-Type': 'application/json' } : undefined,
    body: body ? JSON.stringify(body) : undefined,
  });
  return res.json();
}

async function getJSON(url) {
  const res = await fetch(url);
  return res.json();
}

function setDot(el, on) {
  el.classList.toggle('on', on);
  el.classList.toggle('off', !on);
}

// ── Status polling ──────────────────────────────────────────
let lastPositionsFetch = 0;

async function pollStatus() {
  let s;
  try {
    s = await getJSON('/api/status');
  } catch (e) {
    $('#status-text').textContent = '(cannot reach web UI server)';
    return;
  }

  $('#status-text').textContent = s.status_text || '(no status)';

  setDot($('#dot-robot'), s.robot_running);
  setDot($('#dot-pipeline'), s.pipeline_running);
  $('#btn-robot-start').disabled = s.robot_running;
  $('#btn-robot-stop').disabled = !s.robot_running;
  $('#btn-pipeline-start').disabled = s.pipeline_running;
  $('#btn-pipeline-stop').disabled = !s.pipeline_running;

  $('#btn-execute').disabled = !s.pipeline_running || s.busy;
  $('#busy-indicator').textContent = s.busy ? 'Task in progress…' : '';
  $('#busy-indicator').classList.toggle('busy', !!s.busy);

  const color = s.detected_color || 'unknown';
  $('#color-text').textContent = color;
  const swatch = $('#color-swatch');
  swatch.className = 'swatch' + (['red', 'green', 'blue'].includes(color) ? ' swatch-' + color : '');

  $('#pose-text').textContent = s.detected_pose
    ? `x=${s.detected_pose[0].toFixed(3)}  y=${s.detected_pose[1].toFixed(3)}  z=${s.detected_pose[2].toFixed(3)}`
    : '—';

  $('#result-text').textContent = s.last_result === null ? '—' : (s.last_result ? 'SUCCESS' : 'FAILED');
}

// ── Launch controls ─────────────────────────────────────────
function wireLaunchButtons(prefix, startUrl, stopUrl) {
  $(`#btn-${prefix}-start`).addEventListener('click', async () => {
    $(`#btn-${prefix}-start`).disabled = true;
    const r = await postJSON(startUrl);
    if (!r.ok) alert(r.message);
    pollStatus();
  });
  $(`#btn-${prefix}-stop`).addEventListener('click', async () => {
    $(`#btn-${prefix}-stop`).disabled = true;
    const r = await postJSON(stopUrl);
    if (!r.ok) alert(r.message);
    pollStatus();
  });
}

wireLaunchButtons('robot', '/api/launch/robot/start', '/api/launch/robot/stop');
wireLaunchButtons('pipeline', '/api/launch/pipeline/start', '/api/launch/pipeline/stop');

$('#btn-execute').addEventListener('click', async () => {
  await postJSON('/api/execute');
});

// ── Camera view toggle ───────────────────────────────────────
$('#btn-view-debug').addEventListener('click', () => {
  $('#camera-img').src = '/stream/debug';
  $('#btn-view-debug').classList.add('active');
  $('#btn-view-raw').classList.remove('active');
});
$('#btn-view-raw').addEventListener('click', () => {
  $('#camera-img').src = '/stream/raw';
  $('#btn-view-raw').classList.add('active');
  $('#btn-view-debug').classList.remove('active');
});

// ── Place positions editor ──────────────────────────────────
function rowFor(color) {
  return document.querySelector(`tr[data-color="${color}"]`);
}

function fillPositions(positions) {
  for (const color of ['red', 'green', 'blue']) {
    const xyz = positions[color];
    if (!xyz) continue;
    const row = rowFor(color);
    row.querySelector('.pos-x').value = xyz[0];
    row.querySelector('.pos-y').value = xyz[1];
    row.querySelector('.pos-z').value = xyz[2];
  }
}

async function loadPositions() {
  const r = await getJSON('/api/place_positions');
  fillPositions(r.positions || {});
  $('#positions-msg').textContent = `Loaded from ${r.source}`;
}

$('#btn-reload-positions').addEventListener('click', loadPositions);

$('#btn-save-positions').addEventListener('click', async () => {
  const body = {};
  for (const color of ['red', 'green', 'blue']) {
    const row = rowFor(color);
    const x = parseFloat(row.querySelector('.pos-x').value);
    const y = parseFloat(row.querySelector('.pos-y').value);
    const z = parseFloat(row.querySelector('.pos-z').value);
    if ([x, y, z].every(Number.isFinite)) body[color] = [x, y, z];
  }
  $('#positions-msg').textContent = 'Saving…';
  const r = await postJSON('/api/place_positions', body);
  $('#positions-msg').textContent = r.message;
});

// ── Fullscreen ───────────────────────────────────────────────
$('#btn-fullscreen').addEventListener('click', () => {
  if (!document.fullscreenElement) {
    document.documentElement.requestFullscreen().catch(() => {});
  } else {
    document.exitFullscreen();
  }
});
document.addEventListener('fullscreenchange', () => {
  $('#btn-fullscreen').classList.toggle('active', !!document.fullscreenElement);
});

// ── Clock ────────────────────────────────────────────────────
function tickClock() {
  $('#clock').textContent = new Date().toLocaleTimeString();
}

// ── Boot ─────────────────────────────────────────────────────
loadPositions();
pollStatus();
tickClock();
setInterval(pollStatus, 1000);
setInterval(tickClock, 1000);
