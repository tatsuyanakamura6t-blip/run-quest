// ===== 設定 =====
const DEFAULT_CENTER = [35.6946, 139.7403]; // 位置が取れないときの初期表示（法政大学 市ヶ谷キャンパス付近）
const MAX_ACCURACY_M = 35;   // これより精度が悪いGPS点は使わない
const MIN_STEP_M = 3;        // これより小さい移動はGPSのブレとして無視
const MAX_SPEED_MPS = 12;    // これより速い移動は異常値として無視
const CUR_PACE_WINDOW_MS = 30000; // 現在ペースは直近30秒で計算
const SIM_SPEED_MPS = 1000 / 300; // シミュレーションは 5:00/km で走る

// ===== 地図 =====
const map = L.map('map', { zoomControl: false }).setView(DEFAULT_CENTER, 16);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
  maxZoom: 19,
  attribution: '&copy; OpenStreetMap contributors',
}).addTo(map);

const meMarker = L.marker(DEFAULT_CENTER, {
  icon: L.divIcon({ className: '', html: '<div class="me-dot"></div>', iconSize: [18, 18] }),
});
const accuracyCircle = L.circle(DEFAULT_CENTER, { radius: 0, weight: 0, fillOpacity: 0.12 });
let hasFix = false;
let follow = true;

map.on('dragstart', () => { follow = false; });

// ===== ランの状態 =====
let state = 'idle'; // idle | running | paused | finished
let segments = [];  // 一時停止で区切られた点の配列 [{lat, lng, t}]
let polylines = [];
let distanceM = 0;
let movingMs = 0;       // 一時停止中を除いた経過時間（確定分）
let runningSince = null;
let recent = [];        // 現在ペース用の直近の点 {t, d}
let lastPos = null;

// ===== 時計（シミュレーション時は早送りできるよう抽象化） =====
let simNow = Date.now();
const clock = () => (sim.enabled ? simNow : Date.now());

// ===== 位置情報の入口 =====
function onPosition({ lat, lng, accuracy, t }) {
  lastPos = { lat, lng };
  const latlng = [lat, lng];
  meMarker.setLatLng(latlng);
  accuracyCircle.setLatLng(latlng).setRadius(accuracy);
  if (!hasFix) {
    hasFix = true;
    meMarker.addTo(map);
    accuracyCircle.addTo(map);
    map.setView(latlng, 17);
  } else if (follow) {
    map.panTo(latlng);
  }

  if (state === 'running') {
    recordPoint(lat, lng, accuracy, t);
    setStatus('');
  } else if (state === 'idle') {
    setStatus(accuracy > MAX_ACCURACY_M
      ? `GPS精度が低いです（±${Math.round(accuracy)}m）`
      : `GPS OK（±${Math.round(accuracy)}m）`);
  }
}

function recordPoint(lat, lng, accuracy, t) {
  if (accuracy > MAX_ACCURACY_M) return;

  const seg = segments[segments.length - 1];
  const last = seg[seg.length - 1];
  if (!last) {
    seg.push({ lat, lng, t });
    polylines[polylines.length - 1].addLatLng([lat, lng]);
    return;
  }

  const d = haversine(last, { lat, lng });
  const dt = (t - last.t) / 1000;
  if (d < MIN_STEP_M) return;
  if (dt > 0 && d / dt > MAX_SPEED_MPS) return;

  seg.push({ lat, lng, t });
  polylines[polylines.length - 1].addLatLng([lat, lng]);
  distanceM += d;
  recent.push({ t, d });
}

// ===== 実機GPS =====
let watchId = null;

function startGps() {
  if (!('geolocation' in navigator)) {
    setStatus('このブラウザは位置情報に対応していません');
    return;
  }
  if (!window.isSecureContext) {
    setStatus('位置情報には HTTPS（または localhost）が必要です');
  }
  watchId = navigator.geolocation.watchPosition(
    (p) => onPosition({
      lat: p.coords.latitude,
      lng: p.coords.longitude,
      accuracy: p.coords.accuracy,
      t: p.timestamp,
    }),
    (err) => {
      const msg = {
        1: '位置情報の利用が許可されていません',
        2: '位置情報を取得できません',
        3: '位置情報の取得がタイムアウトしました',
      }[err.code] || err.message;
      setStatus(msg);
    },
    { enableHighAccuracy: true, maximumAge: 0, timeout: 15000 },
  );
}

function stopGps() {
  if (watchId !== null) navigator.geolocation.clearWatch(watchId);
  watchId = null;
}

// ===== シミュレーション（室内テスト用） =====
const sim = { enabled: false, timer: null, heading: Math.random() * 360, pos: null };

function startSim() {
  sim.enabled = true;
  simNow = Date.now();
  sim.pos = lastPos ? { ...lastPos } : { lat: DEFAULT_CENTER[0], lng: DEFAULT_CENTER[1] };
  onPosition({ ...sim.pos, accuracy: 5, t: simNow });
  sim.timer = setInterval(simTick, 1000);
}

function simTick() {
  const speed = Number(document.getElementById('simSpeed').value);
  simNow += 1000 * speed;
  // 向きを少しずつ変えながら進む
  sim.heading += (Math.random() - 0.5) * 30;
  const step = SIM_SPEED_MPS * speed * (0.9 + Math.random() * 0.2);
  sim.pos = movePoint(sim.pos, sim.heading, step);
  onPosition({ ...sim.pos, accuracy: 5, t: simNow });
}

function stopSim() {
  clearInterval(sim.timer);
  sim.enabled = false;
}

// ===== 操作 =====
const startBtn = document.getElementById('startBtn');
const pauseBtn = document.getElementById('pauseBtn');
const finishBtn = document.getElementById('finishBtn');
const simToggle = document.getElementById('simToggle');

startBtn.addEventListener('click', () => {
  if (state === 'finished') resetRun();
  if (state === 'idle' || state === 'paused') {
    beginSegment();
    runningSince = clock();
    state = 'running';
    requestWakeLock();
  }
  render();
});

pauseBtn.addEventListener('click', () => {
  movingMs += clock() - runningSince;
  runningSince = null;
  recent = [];
  state = 'paused';
  setStatus('一時停止中');
  render();
});

finishBtn.addEventListener('click', () => {
  if (!confirm('ランを終了しますか？')) return;
  if (state === 'running') movingMs += clock() - runningSince;
  runningSince = null;
  state = 'finished';
  releaseWakeLock();
  const all = polylines.flatMap((p) => p.getLatLngs());
  if (all.length > 1) map.fitBounds(L.latLngBounds(all), { padding: [40, 40] });
  follow = false;
  setStatus(`おつかれさま！ ${(distanceM / 1000).toFixed(2)} km / ${formatDuration(movingMs)}`);
  render();
});

simToggle.addEventListener('change', () => {
  if (state === 'running' || state === 'paused') {
    simToggle.checked = sim.enabled;
    alert('計測中は切り替えできません');
    return;
  }
  if (simToggle.checked) { stopGps(); startSim(); }
  else { stopSim(); startGps(); }
});

document.getElementById('recenterBtn').addEventListener('click', () => {
  follow = true;
  if (lastPos) map.setView([lastPos.lat, lastPos.lng], Math.max(map.getZoom(), 16));
});

function beginSegment() {
  // スタート地点＝今いる場所から線を引き始める
  const first = lastPos ? [{ ...lastPos, t: clock() }] : [];
  segments.push(first);
  polylines.push(L.polyline(first.map((p) => [p.lat, p.lng]), { color: '#3ee08f', weight: 6, opacity: 0.9 }).addTo(map));
}

function resetRun() {
  polylines.forEach((p) => p.remove());
  segments = [];
  polylines = [];
  distanceM = 0;
  movingMs = 0;
  recent = [];
  follow = true;
  state = 'idle';
}

// ===== 画面更新 =====
function render() {
  startBtn.hidden = state === 'running';
  startBtn.textContent = { idle: 'スタート', paused: '再開', finished: '新しいラン' }[state] || 'スタート';
  pauseBtn.hidden = state !== 'running';
  finishBtn.hidden = !(state === 'running' || state === 'paused');
  simToggle.disabled = state === 'running' || state === 'paused';
  updateStats();
}

function updateStats() {
  const elapsed = movingMs + (runningSince ? clock() - runningSince : 0);
  document.getElementById('distance').textContent = (distanceM / 1000).toFixed(2);
  document.getElementById('time').textContent = formatDuration(elapsed);
  document.getElementById('avgPace').textContent =
    distanceM > 10 ? formatPace(elapsed / 1000 / (distanceM / 1000)) : `--'--"`;

  // 現在ペース：直近30秒の移動から計算（最初の点は起点なので距離に含めない）
  const now = clock();
  recent = recent.filter((r) => now - r.t <= CUR_PACE_WINDOW_MS);
  const d = recent.slice(1).reduce((s, r) => s + r.d, 0);
  const span = recent.length > 1 ? (recent[recent.length - 1].t - recent[0].t) / 1000 : 0;
  document.getElementById('curPace').textContent =
    state === 'running' && d > 10 && span > 0 ? formatPace(span / (d / 1000)) : `--'--"`;
}

setInterval(updateStats, 250);

function setStatus(text) {
  document.getElementById('status').textContent = text;
}

// ===== 画面スリープ防止 =====
let wakeLock = null;
async function requestWakeLock() {
  try { wakeLock = await navigator.wakeLock?.request('screen'); } catch { /* 非対応ブラウザは無視 */ }
}
function releaseWakeLock() {
  wakeLock?.release();
  wakeLock = null;
}
document.addEventListener('visibilitychange', () => {
  if (document.visibilityState === 'visible' && state === 'running') requestWakeLock();
});

// ===== ユーティリティ =====
function haversine(a, b) {
  const R = 6371000;
  const toRad = (x) => (x * Math.PI) / 180;
  const dLat = toRad(b.lat - a.lat);
  const dLng = toRad(b.lng - a.lng);
  const h = Math.sin(dLat / 2) ** 2 +
    Math.cos(toRad(a.lat)) * Math.cos(toRad(b.lat)) * Math.sin(dLng / 2) ** 2;
  return 2 * R * Math.asin(Math.sqrt(h));
}

function movePoint(p, headingDeg, meters) {
  const R = 6371000;
  const h = (headingDeg * Math.PI) / 180;
  const dLat = (meters * Math.cos(h)) / R;
  const dLng = (meters * Math.sin(h)) / (R * Math.cos((p.lat * Math.PI) / 180));
  return { lat: p.lat + (dLat * 180) / Math.PI, lng: p.lng + (dLng * 180) / Math.PI };
}

function formatDuration(ms) {
  const s = Math.floor(ms / 1000);
  const h = Math.floor(s / 3600);
  const m = Math.floor((s % 3600) / 60);
  const sec = s % 60;
  const mmss = `${String(m).padStart(2, '0')}:${String(sec).padStart(2, '0')}`;
  return h > 0 ? `${h}:${mmss}` : mmss;
}

function formatPace(secPerKm) {
  if (!isFinite(secPerKm) || secPerKm > 3600) return `--'--"`;
  const m = Math.floor(secPerKm / 60);
  const s = Math.round(secPerKm % 60);
  return `${m}'${String(s).padStart(2, '0')}"`;
}

// ===== 起動 =====
render();
startGps();
