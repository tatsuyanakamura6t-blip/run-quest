// ===== 設定 =====
const DEFAULT_CENTER = [35.6946, 139.7403]; // 位置が取れないときの初期表示（法政大学 市ヶ谷キャンパス付近）
const MAX_ACCURACY_M = 35;   // これより精度が悪いGPS点は使わない
const MIN_STEP_M = 3;        // これより小さい移動はGPSのブレとして無視
const MAX_SPEED_MPS = 12;    // これより速い移動は異常値として無視
const CUR_PACE_WINDOW_MS = 30000; // 現在ペースは直近30秒で計算
const SIM_SPEED_MPS = 1000 / 300; // シミュレーションは 5:00/km で走る
const KALMAN_Q = 1;          // 平滑化の強さ（小さいほど強くなめらか、大きいほど急な動きに素早く反応）
const KALMAN_RESET_MS = 60000; // これ以上GPSが途切れたら平滑化をリセット
const CURVE_STEP_M = 2;      // 曲線を描くときの分割間隔（m）
// コースの自動生成
const ROUTER_URL = 'https://routing.openstreetmap.de/routed-foot/route/v1/foot/'; // 徒歩用の経路検索（OSRM・APIキー不要）
const COURSE_ROAD_FACTOR = 1.25; // 道は直線より遠回りになる分の見込み（円周 × この値 ≒ 実際の距離）
const COURSE_TOLERANCE = 0.05;   // 目標距離との差がこの割合以内なら完成
const COURSE_TRIES = 4;          // 経路検索をやり直す最大回数
const COURSE_SNAP_M = 40;        // コースからこれ以上離れていたら進み具合を更新しない
const COURSE_LOOKAHEAD_M = 150;  // 進み具合を探す範囲（今の位置より先、m）
const COURSE_DONE_M = 30;        // 残りがこれ以下になったら完走
const DEFAULT_PACE_SEC = 360;    // 目標ペースの初期値（6:00/km）

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
const kalman = createKalman(KALMAN_Q);

function onPosition({ lat: rawLat, lng: rawLng, accuracy, t }) {
  // GPSのブレを平滑化した位置を、以降すべて（マーカー・軌跡・距離）で使う
  const { lat, lng } = kalman.update(rawLat, rawLng, accuracy, t);
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
    updateCourseProgress(lastPos);
    if (!courseDone) setStatus('');
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
    redrawCurrentLine();
    return;
  }

  const d = haversine(last, { lat, lng });
  const dt = (t - last.t) / 1000;
  if (d < MIN_STEP_M) return;
  if (dt > 0 && d / dt > MAX_SPEED_MPS) return;

  seg.push({ lat, lng, t });
  redrawCurrentLine();
  distanceM += d;
  recent.push({ t, d });
}

// 今走っている区間の線を、記録した点を通る曲線で描き直す
function redrawCurrentLine() {
  const seg = segments[segments.length - 1];
  polylines[polylines.length - 1].setLatLngs(smoothPath(seg));
}

// ===== 軌跡の平滑化 =====
// カルマンフィルタ（等速モデル）：位置と速度を同時に推定してGPSのブレを抑える
// ・GPSの精度(accuracy)が悪い点ほど弱く反映する
// ・速度も推定するので、走っている人に対して位置が遅れにくい
// 計算は「スタート地点からの東西・南北のメートル」で行い、最後に緯度経度へ戻す
function createKalman(q) {
  let origin = null;
  let lastT;
  let ax, ay; // 東西(x)・南北(y)それぞれの { x: 位置, v: 速度, P: 誤差の大きさ(2x2) }

  const toRad = (x) => (x * Math.PI) / 180;
  const R = 6371000;

  function axisUpdate(s, z, r, dt) {
    // 予測：速度ぶん進める
    s.x += s.v * dt;
    const [[a, b], [c, d]] = s.P;
    const P00 = a + dt * (b + c) + dt * dt * d + (q * dt ** 3) / 3;
    const P01 = b + dt * d + (q * dt * dt) / 2;
    const P10 = c + dt * d + (q * dt * dt) / 2;
    const P11 = d + q * dt;
    // 補正：GPSの値に寄せる（寄せる強さ k はGPSの精度で決まる）
    const k0 = P00 / (P00 + r);
    const k1 = P10 / (P00 + r);
    const y = z - s.x;
    s.x += k0 * y;
    s.v += k1 * y;
    s.P = [[(1 - k0) * P00, (1 - k0) * P01], [P10 - k1 * P00, P11 - k1 * P01]];
  }

  return {
    update(lat, lng, accuracy, t) {
      const r = Math.max(accuracy, 1) ** 2;
      if (!origin || t - lastT > KALMAN_RESET_MS) {
        origin = { lat, lng };
        lastT = t;
        ax = { x: 0, v: 0, P: [[r, 0], [0, 25]] };
        ay = { x: 0, v: 0, P: [[r, 0], [0, 25]] };
        return { lat, lng };
      }
      const cos = Math.cos(toRad(origin.lat));
      const dt = (t - lastT) / 1000;
      lastT = t;
      if (dt > 0) {
        axisUpdate(ax, toRad(lng - origin.lng) * R * cos, r, dt);
        axisUpdate(ay, toRad(lat - origin.lat) * R, r, dt);
      }
      return {
        lat: origin.lat + ((ay.x / R) * 180) / Math.PI,
        lng: origin.lng + ((ax.x / (R * cos)) * 180) / Math.PI,
      };
    },
  };
}

// Catmull-Rom スプライン（centripetal）：すべての点を通るなめらかな曲線の座標列を返す
// centripetal 方式は急カーブでも線がループしたり膨らみすぎたりしにくい
function smoothPath(pts) {
  if (pts.length < 3) return pts.map((p) => [p.lat, p.lng]);
  // 両端は「反対側に折り返した仮の点」を補って曲線を作る
  const mirror = (a, b) => ({ lat: 2 * a.lat - b.lat, lng: 2 * a.lng - b.lng });
  const ext = [mirror(pts[0], pts[1]), ...pts, mirror(pts[pts.length - 1], pts[pts.length - 2])];

  const out = [[pts[0].lat, pts[0].lng]];
  for (let i = 1; i < ext.length - 2; i++) {
    const [p0, p1, p2, p3] = [ext[i - 1], ext[i], ext[i + 1], ext[i + 2]];
    const n = Math.min(10, Math.max(1, Math.ceil(haversine(p1, p2) / CURVE_STEP_M)));
    for (let j = 1; j <= n; j++) out.push(catmullRom(p0, p1, p2, p3, j / n));
  }
  return out;
}

function catmullRom(p0, p1, p2, p3, u) {
  const knot = (a, b) => Math.max(Math.sqrt(haversine(a, b)), 1e-6);
  const t1 = knot(p0, p1);
  const t2 = t1 + knot(p1, p2);
  const t3 = t2 + knot(p2, p3);
  const t = t1 + (t2 - t1) * u;
  const lerp = (a, b, ta, tb) => {
    const w = (t - ta) / (tb - ta);
    return { lat: a.lat + (b.lat - a.lat) * w, lng: a.lng + (b.lng - a.lng) * w };
  };
  const a1 = lerp(p0, p1, 0, t1);
  const a2 = lerp(p1, p2, t1, t2);
  const a3 = lerp(p2, p3, t2, t3);
  const b1 = lerp(a1, a2, 0, t2);
  const b2 = lerp(a2, a3, t1, t3);
  const c = lerp(b1, b2, t1, t2);
  return [c.lat, c.lng];
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
const sim = { enabled: false, timer: null, heading: Math.random() * 360, pos: null, courseD: 0 };

function startSim() {
  sim.enabled = true;
  simNow = Date.now();
  sim.pos = lastPos ? { ...lastPos } : { lat: DEFAULT_CENTER[0], lng: DEFAULT_CENTER[1] };
  if (course) { sim.courseD = 0; sim.pos = { ...course.points[0] }; }
  onPosition({ ...sim.pos, accuracy: 5, t: simNow });
  sim.timer = setInterval(simTick, 1000);
}

function simTick() {
  const speed = Number(document.getElementById('simSpeed').value);
  simNow += 1000 * speed;
  if (course) {
    // コースがあるときは、計測中だけ目標ペースでコースの上を走る
    if (state === 'running') {
      const step = (1000 / course.paceSec) * speed * (0.9 + Math.random() * 0.2);
      sim.courseD = Math.min(course.lengthM, sim.courseD + step);
    }
    sim.pos = pointAlongCourse(sim.courseD);
  } else {
    // コースがないときは、向きを少しずつ変えながら進む
    sim.heading += (Math.random() - 0.5) * 30;
    const step = SIM_SPEED_MPS * speed * (0.9 + Math.random() * 0.2);
    sim.pos = movePoint(sim.pos, sim.heading, step);
  }
  onPosition({ ...sim.pos, accuracy: 5, t: simNow });
}

function stopSim() {
  clearInterval(sim.timer);
  sim.enabled = false;
}

// ===== コースの自動生成 =====
// 距離（または 時間×目標ペース）を入れると、今いる場所から出発して戻ってくる周回コースを作る。
// しくみ：スタート地点を通る円を描き、円周上の3点を「経由地」にして、徒歩の経路検索で道に沿ったルートにする。
//         できたルートの距離が目標とずれていたら、円の大きさを比で直して検索し直す。
// course は後で「並走者（AIコーチ）」や「コインを置く場所」にも使う。
let course = null; // { points: [{lat,lng}], cum: [スタートからの距離m], lengthM, targetM, paceSec }
let courseLayer = null;
let courseProgressM = 0; // コース上をスタートから何m進んだか
let courseDone = false;
let planMode = 'distance'; // distance | time

async function generateCourse(targetM, paceSec) {
  const start = lastPos || { lat: DEFAULT_CENTER[0], lng: DEFAULT_CENTER[1] };
  const heading = Math.random() * 360; // 毎回ちがう向きに作る
  let radius = targetM / (2 * Math.PI * COURSE_ROAD_FACTOR);
  let best = null;

  for (let i = 0; i < COURSE_TRIES; i++) {
    const center = movePoint(start, heading, radius);
    const back = heading + 180; // 円の中心から見たスタート地点の向き
    const via = [90, 180, 270].map((a) => movePoint(center, back + a, radius));
    const route = await fetchRoute([start, ...via, start]);
    if (!best || Math.abs(route.lengthM - targetM) < Math.abs(best.lengthM - targetM)) best = route;
    if (Math.abs(route.lengthM - targetM) / targetM <= COURSE_TOLERANCE) break;
    radius *= targetM / route.lengthM; // 長すぎたら円を小さく、短すぎたら大きく
  }
  return { ...best, targetM, paceSec };
}

async function fetchRoute(points) {
  const coords = points.map((p) => `${p.lng.toFixed(6)},${p.lat.toFixed(6)}`).join(';');
  const res = await fetch(`${ROUTER_URL}${coords}?overview=full&geometries=geojson`);
  if (!res.ok) throw new Error(`経路検索に失敗しました（${res.status}）`);
  const data = await res.json();
  if (data.code !== 'Ok' || !data.routes?.length) throw new Error('経路が見つかりませんでした');
  const pts = data.routes[0].geometry.coordinates.map(([lng, lat]) => ({ lat, lng }));
  const cum = [0];
  for (let i = 1; i < pts.length; i++) cum.push(cum[i - 1] + haversine(pts[i - 1], pts[i]));
  return { points: pts, cum, lengthM: cum[cum.length - 1] };
}

function drawCourse() {
  courseLayer?.remove();
  courseLayer = null;
  if (!course) return;
  const line = course.points.map((p) => [p.lat, p.lng]);
  courseLayer = L.layerGroup([
    L.polyline(line, { color: '#5aa9ff', weight: 5, opacity: 0.85, dashArray: '2 10', lineCap: 'round' }),
    L.marker(line[0], {
      icon: L.divIcon({ className: '', html: '<div class="start-flag">S</div>', iconSize: [22, 22] }),
    }),
  ]).addTo(map);
  courseLayer.eachLayer((l) => l.bringToBack?.()); // 走った線より下に描く
}

// コース上の、スタートから d m の地点（シミュレーションと、後で並走者に使う）
function pointAlongCourse(d) {
  const { points, cum } = course;
  if (d <= 0) return points[0];
  if (d >= course.lengthM) return points[points.length - 1];
  let i = 1;
  while (cum[i] < d) i++;
  const w = (d - cum[i - 1]) / (cum[i] - cum[i - 1] || 1);
  const a = points[i - 1];
  const b = points[i];
  return { lat: a.lat + (b.lat - a.lat) * w, lng: a.lng + (b.lng - a.lng) * w };
}

// 今の位置がコースのどこにあたるかを、今いる所より少し先までの範囲で探す（周回コースで終点に飛ばないように）
// コースの点と点の間（線分）に垂線を下ろして、いちばん近い所を「進んだ距離」にする
function updateCourseProgress(pos) {
  if (!course) return;
  const { points, cum } = course;
  let bestM = courseProgressM;
  let bestD = Infinity;
  let j = 0;
  while (j < points.length - 2 && cum[j + 1] < courseProgressM) j++; // 今いる線分から探し始める
  for (; j < points.length - 1 && cum[j] <= courseProgressM + COURSE_LOOKAHEAD_M; j++) {
    const { dist, along } = projectOnSegment(pos, points[j], points[j + 1]);
    const m = cum[j] + along;
    if (dist < bestD && m >= courseProgressM) { bestD = dist; bestM = m; }
  }
  if (bestD <= COURSE_SNAP_M) courseProgressM = bestM;
  if (!courseDone && course.lengthM - courseProgressM <= COURSE_DONE_M) {
    courseDone = true;
    setStatus('コース完走！ おつかれさま');
  }
}

// 点 p から線分 a-b への距離と、a から垂線の足までの距離（m）。狭い範囲なので平面として計算する
function projectOnSegment(p, a, b) {
  const R = 6371000;
  const cos = Math.cos((a.lat * Math.PI) / 180);
  const toXY = (q) => ({ x: ((q.lng - a.lng) * Math.PI / 180) * R * cos, y: ((q.lat - a.lat) * Math.PI / 180) * R });
  const B = toXY(b);
  const P = toXY(p);
  const len2 = B.x * B.x + B.y * B.y;
  const u = len2 > 0 ? Math.min(1, Math.max(0, (P.x * B.x + P.y * B.y) / len2)) : 0;
  return { dist: Math.hypot(P.x - u * B.x, P.y - u * B.y), along: u * Math.sqrt(len2) };
}

const planValue = document.getElementById('planValue');
const planPace = document.getElementById('planPace');
const planBtn = document.getElementById('planBtn');
const clearPlanBtn = document.getElementById('clearPlanBtn');

// 目標ペースの選択肢 4:00〜9:00/km（30秒ごと）
for (let sec = 240; sec <= 540; sec += 30) {
  planPace.add(new Option(`${formatPace(sec)}/km`, sec, false, sec === DEFAULT_PACE_SEC));
}

document.querySelectorAll('.seg button').forEach((b) => b.addEventListener('click', () => {
  planMode = b.dataset.mode;
  document.querySelectorAll('.seg button').forEach((x) => x.classList.toggle('on', x === b));
  const isTime = planMode === 'time';
  document.getElementById('planUnit').textContent = isTime ? '分' : 'km';
  Object.assign(planValue, isTime ? { min: 5, max: 300, step: 5 } : { min: 0.5, max: 42, step: 0.5 });
  planValue.value = isTime ? 30 : 3;
}));

planBtn.addEventListener('click', async () => {
  const v = Number(planValue.value);
  const paceSec = Number(planPace.value);
  const targetM = planMode === 'time' ? (v * 60 / paceSec) * 1000 : v * 1000;
  if (!(targetM >= 300 && targetM <= 42195)) {
    setStatus('距離は 0.3〜42km（時間なら目標ペースで その範囲）にしてください');
    return;
  }
  planBtn.disabled = true;
  planBtn.textContent = '作成中…';
  setStatus('コースを作っています…');
  try {
    course = await generateCourse(targetM, paceSec);
    courseProgressM = 0;
    courseDone = false;
    drawCourse();
    fitToVisibleArea(course.points.map((p) => [p.lat, p.lng]));
    follow = false;
    if (sim.enabled) { sim.courseD = 0; sim.pos = { ...course.points[0] }; }
    setStatus('');
  } catch (e) {
    setStatus(`コースを作れませんでした：${e.message}`);
  } finally {
    planBtn.disabled = false;
    render();
  }
});

clearPlanBtn.addEventListener('click', () => {
  course = null;
  drawCourse();
  render();
});

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
  if (all.length > 1) fitToVisibleArea(all);
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
  polylines.push(L.polyline(first.map((p) => [p.lat, p.lng]), {
    color: '#3ee08f', weight: 6, opacity: 0.9, lineCap: 'round', lineJoin: 'round',
  }).addTo(map));
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
  courseProgressM = 0;
  courseDone = false;
  sim.courseD = 0;
}

// ===== 画面更新 =====
function render() {
  startBtn.hidden = state === 'running';
  startBtn.textContent = { idle: 'スタート', paused: '再開', finished: '新しいラン' }[state] || 'スタート';
  pauseBtn.hidden = state !== 'running';
  finishBtn.hidden = !(state === 'running' || state === 'paused');
  simToggle.disabled = state === 'running' || state === 'paused';
  // コースづくりは走る前（と走り終わった後）だけ
  document.getElementById('planner').hidden = state === 'running' || state === 'paused';
  planBtn.textContent = course ? '別のコース' : 'コースを作る';
  clearPlanBtn.hidden = !course;
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

  // コース情報：走る前は「距離と目安の時間」、走り出したら「残り」
  const info = document.getElementById('courseInfo');
  info.hidden = !course;
  if (course) {
    const km = (course.lengthM / 1000).toFixed(2);
    if (state === 'idle') {
      const etaMs = (course.lengthM / 1000) * course.paceSec * 1000;
      info.textContent = `コース ${km} km ・ 目安 ${formatDuration(etaMs)}（${formatPace(course.paceSec)}/km）`;
    } else {
      const rest = Math.max(0, course.lengthM - courseProgressM) / 1000;
      info.textContent = `コース ${km} km ・ 残り ${rest.toFixed(2)} km`;
    }
  }
}

setInterval(updateStats, 250);

// 上の計測値と下の操作パネルに隠れない範囲に、線全体が収まるように地図を動かす
function fitToVisibleArea(latlngs) {
  const top = document.querySelector('.stats').offsetHeight;
  const bottom = document.querySelector('.controls').offsetHeight;
  map.fitBounds(L.latLngBounds(latlngs), {
    paddingTopLeft: [24, top + 24],
    paddingBottomRight: [24, bottom + 24],
  });
}

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
