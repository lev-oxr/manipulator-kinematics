import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { ConvexGeometry } from 'three/addons/geometries/ConvexGeometry.js';
import { DEFAULTS, appendixB, fk, ik, sources, errorVectors, worstError, jointError } from './kin.js';

const D = Math.PI / 180;
const $ = id => document.getElementById(id);
const f = (v, d = 1) => v.toLocaleString('uk-UA', { minimumFractionDigits: d, maximumFractionDigits: d });
const tok = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
const dark = matchMedia('(prefers-color-scheme: dark)');
const RAMP = ['#cde2fb', '#b7d3f6', '#9ec5f4', '#86b6ef', '#6da7ec', '#5598e7', '#3987e5', '#2a78d6', '#256abf', '#1c5cab', '#184f95', '#104281', '#0d366b'];
const hexRGB = h => [1, 3, 5].map(i => parseInt(h.slice(i, i + 2), 16));
const nice = x => { const e = 10 ** Math.floor(Math.log10(x)); return [5, 2, 1].map(m => m * e).find(v => v <= x); };
const niceUp = x => { const e = 10 ** Math.floor(Math.log10(x)); return [1, 2, 2.5, 5, 10].map(m => m * e).find(v => v >= x * (1 - 1e-9)); };
const clamp = (v, a, b) => Math.min(b, Math.max(a, v));

const S = {
  len: { ...DEFAULTS }, lim: {}, drive: {},
  q: [30 * D, 60 * D, 90 * D],
  elbow: 1,
  target: null, // недосяжна ціль, яку задав користувач
  trajT: null,  // момент t на траєкторії Додатку В, якщо рука зараз на ній
};
const reach = () => S.len.L2 + S.len.L3;
const stepRad = () => 1.8 / S.drive.micro * S.drive.z1 / S.drive.z2 * D;
const dqs = () => Array(3).fill(jointError({ micro: S.drive.micro, gear: S.drive.z1 / S.drive.z2, backlashDeg: S.drive.backlash }));
const errVecs = q => errorVectors(q, S.len, dqs(), S.drive.dL);
// межі кутів (задає користувач) + лікоть не під столом; ВСП під стіл не пускають самі діапазони Z
const inLimits = q => q[1] >= S.lim.t2[0] * D - 1e-9 && q[1] <= S.lim.t2[1] * D + 1e-9
  && q[2] >= S.lim.t3[0] * D - 1e-9 && q[2] <= S.lim.t3[1] * D + 1e-9
  && S.len.L1 + S.len.L2 * Math.sin(q[1]) >= 0;
const msg = t => { $('msg').textContent = t ? '⚠ ' + t : ''; };

// ---------- 3D-сцена: група W має вісь Z вгору, як у дисертації ----------
const view = $('view');
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(devicePixelRatio);
view.prepend(renderer.domElement);
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(38, 1, 1, 5000);
camera.position.set(330, 330, 520);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(60, 140, 0);
controls.update();
controls.saveState();
controls.addEventListener('change', render);
$('resetView').addEventListener('click', () => controls.reset());
// коліщатко гортає сторінку; масштаб — з Ctrl (так само приходить щипок на тачпаді)
view.addEventListener('wheel', e => { if (!e.ctrlKey && !e.metaKey) e.stopPropagation(); }, { capture: true });
scene.add(new THREE.HemisphereLight(0xffffff, 0x777777, 1.8));
const sun = new THREE.DirectionalLight(0xffffff, 1.6);
sun.position.set(300, 600, 250);
scene.add(sun);
let grid = new THREE.Object3D();
const W = new THREE.Group();
W.rotation.x = -Math.PI / 2;
scene.add(W);
W.add(new THREE.AxesHelper(60));

const linkMat = new THREE.MeshStandardMaterial({ roughness: .55, metalness: .1 });
const jointMat = new THREE.MeshStandardMaterial({ roughness: .5, metalness: .1 });
const probeMat = new THREE.MeshStandardMaterial({ roughness: .4 });
const ghostMat = new THREE.MeshBasicMaterial({ transparent: true, opacity: .35, depthWrite: false });
const targetMat = new THREE.MeshBasicMaterial({ wireframe: true });
const CYL = new THREE.CylinderGeometry(1, 1, 1, 28), SPH = new THREE.SphereGeometry(1, 28, 18), UP = new THREE.Vector3(0, 1, 0);

function place(m, a, b) {
  const A = new THREE.Vector3(...a), d = new THREE.Vector3(...b).sub(A);
  m.position.copy(A).addScaledVector(d, .5);
  m.scale.set(m.userData.r, d.length() || 1e-6, m.userData.r);
  if (d.lengthSq() > 0) m.quaternion.setFromUnitVectors(UP, d.normalize());
}
function makeArm(link, joint, probe) {
  const g = new THREE.Group();
  W.add(g);
  const bar = r => { const m = new THREE.Mesh(CYL, link); m.userData.r = r; g.add(m); return m; };
  const ball = (r, mat) => { const m = new THREE.Mesh(SPH, mat); m.scale.setScalar(r); g.add(m); return m; };
  const b1 = bar(14), b2 = bar(9), b3 = bar(7), j2 = ball(13, joint), j3 = ball(11, joint), tip = ball(7, probe);
  return {
    group: g, tip,
    set(q) {
      const { L1, L2 } = S.len, [t1, t2] = q;
      const sh = [0, 0, L1], el = [L2 * Math.cos(t2) * Math.cos(t1), L2 * Math.cos(t2) * Math.sin(t1), L1 + L2 * Math.sin(t2)], p = fk(q, S.len);
      place(b1, [0, 0, 0], sh); place(b2, sh, el); place(b3, el, p);
      j2.position.set(...sh); j3.position.set(...el); tip.position.set(...p);
    },
  };
}
const disk = new THREE.Mesh(new THREE.CylinderGeometry(38, 44, 10, 48), jointMat);
disk.rotation.x = Math.PI / 2;
disk.position.z = 5;
W.add(disk);
const arm = makeArm(linkMat, jointMat, probeMat);
const ghost = makeArm(ghostMat, ghostMat, ghostMat);
const targetMark = new THREE.Mesh(new THREE.SphereGeometry(7, 12, 8), targetMat);
W.add(targetMark);

const tipPx = new THREE.Vector3();
function render() {
  renderer.render(scene, camera);
  arm.tip.getWorldPosition(tipPx).project(camera);
  const lb = $('probeLabel'), x = Math.min((tipPx.x + 1) / 2 * view.clientWidth + 14, view.clientWidth - lb.offsetWidth - 8);
  lb.style.transform = `translate(${x}px, ${(1 - tipPx.y) / 2 * view.clientHeight - 30}px)`;
}

// перетягування ВСП: рух у вертикальній площині руки, θ1 не змінюється
const grip = new THREE.Mesh(SPH, new THREE.MeshBasicMaterial({ visible: false }));
grip.scale.setScalar(16); // ціль для миші більша за саму кульку
W.add(grip);
const ray = new THREE.Raycaster(), ndc = new THREE.Vector2(), armPlane = new THREE.Plane(), hit = new THREE.Vector3(), inv = new THREE.Matrix4();
let dragging = false;
function aim(e) {
  const b = renderer.domElement.getBoundingClientRect();
  ndc.set((e.clientX - b.left) / b.width * 2 - 1, (b.top - e.clientY) / b.height * 2 + 1);
  ray.setFromCamera(ndc, camera);
  return ray.intersectObject(grip).length > 0;
}
view.addEventListener('pointerdown', e => {
  if (e.target !== renderer.domElement || !aim(e)) return;
  e.stopPropagation(); // OrbitControls цю подію не отримає
  dragging = true;
  renderer.domElement.setPointerCapture(e.pointerId);
  renderer.domElement.style.cursor = 'grabbing';
}, { capture: true });
view.addEventListener('pointermove', e => {
  if (e.target !== renderer.domElement) return;
  if (!dragging) { renderer.domElement.style.cursor = aim(e) ? 'grab' : ''; return; }
  aim(e);
  const t1 = S.q[0];
  armPlane.set(new THREE.Vector3(-Math.sin(t1), Math.cos(t1), 0), 0);
  ray.ray.applyMatrix4(inv.copy(W.matrixWorld).invert());
  if (!ray.ray.intersectPlane(armPlane, hit)) return;
  const r = Math.max(0, hit.x * Math.cos(t1) + hit.y * Math.sin(t1));
  setTarget([r * Math.cos(t1), r * Math.sin(t1), Math.max(0, hit.z)]);
});
const drop = () => { dragging = false; renderer.domElement.style.cursor = ''; };
view.addEventListener('pointerup', drop);
view.addEventListener('pointercancel', drop);

// площина-переріз з картою похибки, обертається разом з θ1
const pivot = new THREE.Group();
W.add(pivot);
const plane = new THREE.Mesh(new THREE.PlaneGeometry(1, 1), new THREE.MeshBasicMaterial({ transparent: true, opacity: .85, side: THREE.DoubleSide, depthWrite: false }));
plane.rotation.x = Math.PI / 2;
pivot.add(plane);

// область можливих положень ВСП: опукла оболонка 2^n вершин похибки, збільшена в k разів
const zMesh = new THREE.Mesh(new THREE.BufferGeometry(), new THREE.MeshBasicMaterial({ transparent: true, opacity: .35, depthWrite: false }));
const zEdges = new THREE.LineSegments(new THREE.BufferGeometry(), new THREE.LineBasicMaterial());
W.add(zMesh, zEdges);
function zono(p, vecs, worst) {
  let geo = new THREE.BufferGeometry(), k = 0;
  if ($('ly-zono').checked && worst > 1e-6) {
    k = nice(25 / worst);
    const pts = [];
    for (let m = 0; m < 1 << vecs.length; m++) {
      const v = new THREE.Vector3(...p);
      vecs.forEach((e, i) => v.add(new THREE.Vector3(...e).multiplyScalar(m >> i & 1 ? k : -k)));
      pts.push(v);
    }
    try { geo = new ConvexGeometry(pts); } catch { k = 0; } // вироджена оболонка (r = 0)
  }
  zMesh.geometry.dispose(); zEdges.geometry.dispose();
  zMesh.geometry = geo;
  zEdges.geometry = geo.attributes.position ? new THREE.EdgesGeometry(geo, 1) : new THREE.BufferGeometry();
  $('zoomK').textContent = k || '—';
}

const cloud = new THREE.Points(new THREE.BufferGeometry(), new THREE.PointsMaterial({ size: 2, sizeAttenuation: false, vertexColors: true, transparent: true, opacity: .4, depthWrite: false }));
W.add(cloud);

// ---------- карта похибки в площині (r, Z) ----------
const mapBase = document.createElement('canvas');
let M = null;
const rampIdx = i => { const k = M.n > 1 ? Math.round(i * 12 / (M.n - 1)) : 6; return dark.matches ? 12 - k : k; };
const band = v => Math.min(M.n - 1, Math.floor(v / M.step));

function buildMap() {
  const { L1 } = S.len, R = reach(), z0 = Math.max(0, L1 - R), z1 = L1 + R;
  const res = Math.max(1, Math.ceil(Math.max(R, z1 - z0) / 320)); // мм на піксель
  const w = Math.ceil(R / res), h = Math.ceil((z1 - z0) / res), vals = new Float32Array(w * h).fill(NaN);
  let max = 0;
  for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) {
    const q = ik([(x + .5) * res, 0, z1 - (y + .5) * res], S.len, S.elbow);
    if (!q || !inLimits(q)) continue;
    const e = worstError(errVecs(q));
    vals[y * w + x] = e;
    if (e > max) max = e;
  }
  let step = niceUp(max / 7 || 1), n = Math.max(1, Math.ceil(max / step - 1e-9));
  if ($('lockScale').checked && M) ({ step, n } = M);
  M = { w, h, res, z0, z1, R, vals, max, step, n };
  paintMap();
}
function paintMap() {
  const rgb = Array.from({ length: M.n }, (_, i) => hexRGB(RAMP[rampIdx(i)]));
  const lin = Array.from({ length: M.n }, (_, i) => new THREE.Color(RAMP[rampIdx(i)]));
  mapBase.width = M.w; mapBase.height = M.h;
  const ctx = mapBase.getContext('2d'), img = ctx.createImageData(M.w, M.h);
  M.vals.forEach((v, i) => { if (v === v) img.data.set([...rgb[band(v)], 255], i * 4); });
  ctx.putImageData(img, 0, 0);
  plane.material.map?.dispose();
  plane.material.map = Object.assign(new THREE.CanvasTexture(mapBase), { colorSpace: THREE.SRGBColorSpace });
  plane.material.needsUpdate = true;
  plane.scale.set(M.w * M.res, M.h * M.res, 1);
  plane.position.set(M.w * M.res / 2, 0, M.z1 - M.h * M.res / 2);

  // хмара = та сама карта, обернута навколо осі основи (крок ~10 мм)
  cloud.visible = $('ly-cloud').checked;
  if (cloud.visible) {
    const pos = [], col = [], stride = Math.max(1, Math.round(10 / M.res));
    for (let y = 0; y < M.h; y += stride) for (let x = 0; x < M.w; x += stride) {
      const v = M.vals[y * M.w + x];
      if (v !== v) continue;
      const r = (x + .5) * M.res, z = M.z1 - (y + .5) * M.res, n = Math.max(1, Math.round(2 * Math.PI * r / 10)), c = lin[band(v)];
      for (let k = 0; k < n; k++) { const a = 2 * Math.PI * k / n; pos.push(r * Math.cos(a), r * Math.sin(a), z); col.push(c.r, c.g, c.b); }
    }
    cloud.geometry.dispose();
    cloud.geometry = new THREE.BufferGeometry();
    cloud.geometry.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
    cloud.geometry.setAttribute('color', new THREE.Float32BufferAttribute(col, 3));
  }

  const top = M.max > M.n * M.step ? 'і більше, мм' : f(M.n * M.step, 2) + ' мм'; // шкала зафіксована, а похибка зросла
  $('legend').innerHTML = rgb.map((_, i) => `<div><i style="background:${RAMP[rampIdx(i)]}"></i><span>${f(i * M.step, 2)}</span></div>`).join('')
    + `<div class="end"><span>${top}</span></div>`;
  $('legend3d').innerHTML = `|Δr| 0 ${rgb.map((_, i) => `<i style="background:${RAMP[rampIdx(i)]}"></i>`).join('')} ${top}`;
  $('mapMax').textContent = $('zoneMax').textContent = f(M.max, 2);
}

const mc = $('map'), mctx = mc.getContext('2d'), PAD = { l: 44, r: 10, t: 10, b: 34 };
function drawMap() {
  if (!M) return;
  const dpr = devicePixelRatio || 1, asp = M.R / (M.z1 - M.z0);
  const pw = Math.max(120, Math.min($('mapwrap').clientWidth - PAD.l - PAD.r, 540 * asp)), ph = pw / asp;
  const cw = Math.round(pw + PAD.l + PAD.r), ch = Math.round(ph + PAD.t + PAD.b);
  if (mc.width !== cw * dpr || mc.height !== ch * dpr) {
    mc.width = cw * dpr; mc.height = ch * dpr; mc.style.width = cw + 'px'; mc.style.height = ch + 'px';
  }
  const c = mctx, k = pw / M.R, sx = r => PAD.l + r * k, sy = z => PAD.t + (M.z1 - z) * k;
  Object.assign(M, { pw, ph, k });
  c.setTransform(dpr, 0, 0, dpr, 0, 0);
  c.clearRect(0, 0, cw, ch);
  c.font = '12px system-ui, sans-serif';
  c.lineWidth = 1;
  c.strokeStyle = tok('--grid'); c.fillStyle = tok('--muted');
  c.textAlign = 'center'; c.textBaseline = 'top';
  for (let r = 0; r <= M.R + 1e-9; r += 50) {
    c.beginPath(); c.moveTo(sx(r) + .5, PAD.t); c.lineTo(sx(r) + .5, PAD.t + ph); c.stroke();
    c.fillText(r, sx(r), PAD.t + ph + 6);
  }
  c.textAlign = 'right'; c.textBaseline = 'middle';
  for (let z = Math.ceil(M.z0 / 50) * 50; z <= M.z1 + 1e-9; z += 50) {
    c.beginPath(); c.moveTo(PAD.l, sy(z) + .5); c.lineTo(PAD.l + pw, sy(z) + .5); c.stroke();
    c.fillText(z, PAD.l - 6, sy(z));
  }
  c.textAlign = 'right'; c.textBaseline = 'bottom';
  c.fillText('r, мм', PAD.l + pw, ch - 1);
  c.textAlign = 'left'; c.textBaseline = 'top';
  c.fillText('Z, мм', 4, 0);

  c.save();
  c.beginPath(); c.rect(PAD.l, PAD.t, pw, ph); c.clip();
  c.imageSmoothingEnabled = false;
  c.drawImage(mapBase, sx(0), sy(M.z1), M.w * M.res * k, M.h * M.res * k);
  const { L1, L2, L3 } = S.len;
  c.setLineDash([4, 4]); c.strokeStyle = tok('--muted');
  for (const rad of [L2 + L3, Math.abs(L2 - L3)]) { c.beginPath(); c.arc(sx(0), sy(L1), rad * k, 0, 2 * Math.PI); c.stroke(); }
  c.setLineDash([]);
  if (S.trajT !== null) { // траєкторія Додатку В у площині r–Z: які смуги похибки вона перетинає
    c.strokeStyle = tok('--probe'); c.lineWidth = 2;
    c.beginPath();
    TT.forEach((t, i) => { const q = fk(appendixB(t), S.len); c[i ? 'lineTo' : 'moveTo'](sx(Math.hypot(q[0], q[1])), sy(q[2])); });
    c.stroke();
  }
  const armPath = (q, dashed) => {
    const [, t2, t3] = q, e = [L2 * Math.cos(t2), L1 + L2 * Math.sin(t2)], p = [e[0] + L3 * Math.cos(t2 - t3), e[1] + L3 * Math.sin(t2 - t3)];
    c.setLineDash(dashed ? [5, 4] : []);
    c.lineWidth = dashed ? 1.5 : 3; c.lineCap = 'round'; c.lineJoin = 'round';
    c.strokeStyle = tok(dashed ? '--muted' : '--ink');
    c.beginPath(); c.moveTo(sx(0), sy(0)); c.lineTo(sx(0), sy(L1)); c.lineTo(sx(e[0]), sy(e[1])); c.lineTo(sx(p[0]), sy(p[1])); c.stroke();
    c.setLineDash([]);
    return p;
  };
  if (ghost.group.visible) armPath(ghost.q, true);
  const p = armPath(S.q, false);
  c.beginPath(); c.arc(sx(p[0]), sy(p[1]), 5, 0, 2 * Math.PI);
  c.fillStyle = tok('--probe'); c.fill();
  c.lineWidth = 2; c.strokeStyle = tok('--surface'); c.stroke();
  if (S.target) {
    const tr = Math.hypot(S.target[0], S.target[1]), tz = S.target[2];
    c.strokeStyle = tok('--warn'); c.lineWidth = 2;
    c.beginPath(); c.moveTo(sx(tr) - 6, sy(tz) - 6); c.lineTo(sx(tr) + 6, sy(tz) + 6); c.moveTo(sx(tr) + 6, sy(tz) - 6); c.lineTo(sx(tr) - 6, sy(tz) + 6); c.stroke();
  }
  c.restore();
}
function mapPoint(e) {
  const b = mc.getBoundingClientRect(), x = e.clientX - b.left, y = e.clientY - b.top;
  const r = (x - PAD.l) / M.k, z = M.z1 - (y - PAD.t) / M.k;
  return { x, y, r, z, inside: r >= 0 && r <= M.R && z >= M.z0 && z <= M.z1 };
}
mc.addEventListener('pointermove', e => {
  const tip = $('tip'), m = mapPoint(e);
  if (!m.inside) { tip.hidden = true; return; }
  const q = ik([m.r, 0, m.z], S.len, S.elbow), ok = q && inLimits(q);
  tip.innerHTML = `r = ${f(m.r)} мм, Z = ${f(m.z)} мм<br>` + (ok
    ? `<b>|Δr| ≤ ${f(worstError(errVecs(q)), 2)} мм</b><br>θ₂ = ${f(q[1] / D)}°, θ₃ = ${f(q[2] / D)}°`
    : 'недосяжно в цій конфігурації');
  tip.hidden = false;
  tip.style.left = (m.x > PAD.l + M.pw / 2 ? m.x - tip.offsetWidth - 12 : m.x + 12) + 'px';
  tip.style.top = m.y + 12 + 'px';
});
mc.addEventListener('pointerleave', () => { $('tip').hidden = true; });
mc.addEventListener('click', e => {
  const m = mapPoint(e);
  if (m.inside) setTarget([m.r * Math.cos(S.q[0]), m.r * Math.sin(S.q[0]), m.z], true);
});

// ---------- панель ----------
const JOINTS = [['t1', 'θ₁'], ['t2', 'θ₂'], ['t3', 'θ₃']], CART = [['X', 'X'], ['Y', 'Y'], ['Z', 'Z']];
for (const [box, list, unit] of [['jointRows', JOINTS, '°'], ['cartRows', CART, 'мм']])
  for (const [k, label] of list)
    $(box).insertAdjacentHTML('beforeend', `<div class="row"><label for="in-${k}">${label}</label><input type="range" id="in-${k}"><input type="number" id="num-${k}" step="0.1" aria-label="${label}, ${unit}"><span>${unit}</span></div>`);
// точне значення з клавіатури йде тим самим шляхом, що й повзунок
[...JOINTS, ...CART].forEach(([k]) => $('num-' + k).addEventListener('change', e => {
  $('in-' + k).value = e.target.value;
  $('in-' + k).dispatchEvent(new Event('input'));
}));

function setRanges() {
  const R = reach(), set = (k, min, max) => Object.assign($('in-' + k), { min, max, step: .1 });
  set('t1', -180, 180); set('t2', ...S.lim.t2); set('t3', ...S.lim.t3);
  set('X', -R, R); set('Y', -R, R); set('Z', 0, S.len.L1 + R);
}
JOINTS.forEach(([k], j) => $('in-' + k).addEventListener('input', e => {
  cancelAnimationFrame(anim); leaveTraj();
  S.q[j] = +e.target.value * D;
  S.target = null; msg('');
  if (S.q[2]) setElbow(Math.sign(S.q[2]), false);
  update();
}));
CART.forEach(([k]) => $('in-' + k).addEventListener('input', () => setTarget(CART.map(([c]) => +$('in-' + c).value))));

// плавний перехід у просторі суглобів — видно, як рука перебудовується
let anim = 0;
function animateTo(q) {
  cancelAnimationFrame(anim);
  const from = [...S.q], d = q.map((v, i) => v - from[i]), t0 = performance.now();
  d[0] = Math.atan2(Math.sin(d[0]), Math.cos(d[0])); // основа — коротшим шляхом
  const tick = now => {
    const k = Math.min(1, (now - t0) / 450), e = k < .5 ? 2 * k * k : 1 - (2 - 2 * k) ** 2 / 2;
    S.q = k < 1 ? from.map((v, i) => v + d[i] * e) : [...q];
    update();
    if (k < 1) anim = requestAnimationFrame(tick);
  };
  anim = requestAnimationFrame(tick);
}

function setTarget(t, smooth = false) {
  cancelAnimationFrame(anim); leaveTraj();
  const q = ik(t, S.len, S.elbow);
  if (q && inLimits(q)) {
    S.target = null; msg('');
    if (smooth) return animateTo(q);
    S.q = q;
  } else {
    S.target = t;
    const alt = ik(t, S.len, -S.elbow);
    msg(!q ? 'Точка поза робочою зоною: відстань від плеча більша за L₂ + L₃ або менша за |L₂ − L₃|.'
      : alt && inLimits(alt) ? 'Точка досяжна лише в іншій конфігурації ліктя.'
        : 'Точка недосяжна: лікоть опинився б нижче столу або кут вийшов би за межі.');
  }
  update();
}
function setElbow(e, move = true) {
  if (move) {
    const q = ik(fk(S.q, S.len), S.len, e);
    if (!q || !inLimits(q)) return msg(`У конфігурації «лікоть ${e > 0 ? 'вгору' : 'вниз'}» лікоть опинився б нижче столу або кут вийшов би за межі.`);
    S.target = null;
    animateTo(q);
  }
  document.querySelectorAll('[data-elbow]').forEach(b => b.setAttribute('aria-pressed', +b.dataset.elbow === e));
  if (e !== S.elbow) { S.elbow = e; buildMap(); }
}
document.querySelectorAll('[data-elbow]').forEach(b => b.addEventListener('click', () => { msg(''); leaveTraj(); setElbow(+b.dataset.elbow); update(); }));

const NAMES = ['θ₁ основа', 'θ₂ плече', 'θ₃ лікоть', 'ΔL ланки'];
function bars(vecs) {
  const vals = [0, 1, 2].map(i => Math.hypot(...vecs[i])).concat(vecs.length > 3 ? worstError(vecs.slice(3)) : 0);
  const max = Math.max(...vals) || 1;
  $('bars').innerHTML = vals.map((v, i) => `<span>${NAMES[i]}</span><span class="track"><i style="width:${v / max * 100}%"></i></span><span>${f(v, 2)}</span>`).join('');
}
function jacobian() {
  const { J } = sources(S.q, S.len), [a, b, c] = J, { L2, L3 } = S.len;
  const det = a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0]);
  $('jac').innerHTML = ['X', 'Y', 'Z'].map((n, i) => `<tr><th>∂${n}</th>${J.map(col => `<td>${f(col[i], 1)}</td>`).join('')}</tr>`).join('');
  $('det').textContent = f(Math.abs(det) / (L2 * L3 * (L2 + L3)), 3);
}

// ---------- траєкторія з Додатку В ----------
const TT = Array.from({ length: 101 }, (_, i) => i / 100);
const TCH = [['X(t)', 1], ['Y(t)', 1], ['Z(t)', 1], ['|Δr|(t)', 2]]; // назва, знаків після коми
$('trajCharts').innerHTML = TCH.map(([n], k) => `<div class="tchart"><div class="th"><span>${n}, мм</span><b id="tv${k}">—</b></div><canvas id="tc${k}" role="img" aria-label="Графік ${n}"></canvas></div>`).join('');
// трубка, а не лінія: WebGL-лінії завжди 1 px. Прозора й пізніше за площину карти, щоб не ховалася за нею
const pathLine = new THREE.Mesh(new THREE.BufferGeometry(), new THREE.MeshBasicMaterial({ transparent: true, opacity: .9 }));
pathLine.renderOrder = 10;
W.add(pathLine);
let traj = null, hoverT = null, trajAnim = 0;

function buildTraj() {
  const P = TT.map(t => fk(appendixB(t), S.len));
  traj = [0, 1, 2].map(i => P.map(p => p[i])).concat([TT.map(t => worstError(errVecs(appendixB(t))))]);
  pathLine.geometry.dispose();
  pathLine.geometry = new THREE.TubeGeometry(new THREE.CatmullRomCurve3(P.map(p => new THREE.Vector3(...p))), 100, 1.6, 8);
}
function setTrajT(t) {
  cancelAnimationFrame(anim);
  S.target = null; msg('');
  S.trajT = t; S.q = appendixB(t);
  setElbow(1, false); // θ3 > 0 на всій траєкторії
  update();
}
function pauseTraj() { cancelAnimationFrame(trajAnim); trajAnim = 0; $('trajPlay').textContent = '▶ Відтворити'; }
function leaveTraj() { pauseTraj(); S.trajT = null; } // користувач узяв керування на себе
const scrub = t => { pauseTraj(); setTrajT(t); };
$('trajPlay').addEventListener('click', () => {
  if (trajAnim) return pauseTraj();
  const t0 = S.trajT === null || S.trajT >= 1 ? 0 : S.trajT, start = performance.now();
  $('trajPlay').textContent = '❚❚ Пауза';
  const tick = now => {
    const t = Math.min(1, t0 + (now - start) / 4000); // 1 с моделі = 4 с на екрані
    setTrajT(t);
    if (t < 1) trajAnim = requestAnimationFrame(tick); else pauseTraj();
  };
  trajAnim = requestAnimationFrame(tick);
});
$('trajT').addEventListener('input', e => scrub(+e.target.value));

const sample = (vals, t) => { const x = t * 100, i = Math.min(99, Math.floor(x)); return vals[i] + (vals[i + 1] - vals[i]) * (x - i); };
const CL = 40, CR = 8; // поля графіків
function drawCharts() {
  if (!traj) return;
  const tc = hoverT ?? S.trajT, dpr = devicePixelRatio || 1;
  TCH.forEach(([, d], k) => {
    const cv = $('tc' + k), vals = traj[k], last = k === TCH.length - 1, w = cv.clientWidth, h = last ? 84 : 64;
    if (!w) return;
    if (cv.width !== Math.round(w * dpr) || cv.height !== h * dpr) { cv.width = Math.round(w * dpr); cv.height = h * dpr; cv.style.height = h + 'px'; }
    const c = cv.getContext('2d'), T = 6, ph = h - T - (last ? 26 : 6), pw = w - CL - CR;
    let lo = last ? 0 : Math.min(...vals), hi = Math.max(...vals); // похибка — від нуля, щоб не роздувати сотки мм
    if (hi - lo < 1e-9) { lo -= 1; hi += 1; }
    const X = t => CL + t * pw, Y = v => T + (hi - v) / (hi - lo) * ph;
    c.setTransform(dpr, 0, 0, dpr, 0, 0);
    c.clearRect(0, 0, w, h);
    c.font = '11px system-ui, sans-serif'; c.fillStyle = tok('--muted'); c.strokeStyle = tok('--grid'); c.lineWidth = 1;
    c.textAlign = 'right'; c.textBaseline = 'middle';
    for (const v of [lo, hi]) {
      c.beginPath(); c.moveTo(CL, Math.round(Y(v)) + .5); c.lineTo(CL + pw, Math.round(Y(v)) + .5); c.stroke();
      c.fillText(f(v, d === 2 ? 2 : 0), CL - 6, Y(v));
    }
    if (last) {
      c.textBaseline = 'top';
      [0, .25, .5, .75, 1].forEach(t => { c.textAlign = t === 1 ? 'right' : 'center'; c.fillText(t === 1 ? '1 с' : f(t, t % .5 ? 2 : 1), X(t), T + ph + 8); });
    }
    c.strokeStyle = tok('--accent'); c.lineWidth = 2; c.lineJoin = 'round';
    c.beginPath(); vals.forEach((v, i) => c[i ? 'lineTo' : 'moveTo'](X(TT[i]), Y(v))); c.stroke();
    if (tc === null) { $('tv' + k).textContent = '—'; return; }
    const v = sample(vals, tc), x = Math.round(X(tc)) + .5;
    c.strokeStyle = tok(hoverT === null ? '--ink' : '--muted'); c.lineWidth = 1;
    c.beginPath(); c.moveTo(x, T); c.lineTo(x, T + ph); c.stroke();
    c.beginPath(); c.arc(X(tc), Y(v), 4, 0, 2 * Math.PI);
    c.fillStyle = tok('--accent'); c.fill(); c.lineWidth = 2; c.strokeStyle = tok('--surface'); c.stroke();
    $('tv' + k).textContent = f(v, d);
  });
  $('trajT').value = S.trajT ?? 0;
  $('trajTout').value = tc === null ? '—' : f(tc, 2) + ' с';
}
TCH.forEach((_, k) => {
  const cv = $('tc' + k);
  const tAt = e => { const b = cv.getBoundingClientRect(); return Math.round(clamp((e.clientX - b.left - CL) / (b.width - CL - CR), 0, 1) * 100) / 100; };
  cv.addEventListener('pointerdown', e => { cv.setPointerCapture(e.pointerId); scrub(tAt(e)); });
  cv.addEventListener('pointermove', e => { hoverT = tAt(e); if (e.buttons & 1) scrub(hoverT); else drawCharts(); });
  cv.addEventListener('pointerleave', () => { hoverT = null; drawCharts(); });
});
new ResizeObserver(drawCharts).observe($('trajCharts'));

function update() {
  const p = fk(S.q, S.len);
  const show = (k, v) => { $('in-' + k).value = v; if (document.activeElement !== $('num-' + k)) $('num-' + k).value = v.toFixed(1); };
  JOINTS.forEach(([k], j) => show(k, S.q[j] / D));
  CART.forEach(([k], i) => show(k, S.target ? S.target[i] : p[i])); // недосяжна ціль лишається в полях

  arm.set(S.q);
  grip.position.set(...p);
  ghost.q = ik(p, S.len, -(Math.sign(S.q[2]) || S.elbow));
  ghost.group.visible = $('ly-ghost').checked && !!ghost.q && inLimits(ghost.q) && Math.abs(S.q[2]) > 1e-3;
  if (ghost.group.visible) ghost.set(ghost.q);
  targetMark.visible = !!S.target;
  if (S.target) targetMark.position.set(...S.target);
  pivot.rotation.z = S.q[0];
  plane.visible = $('ly-map').checked;

  const vecs = errVecs(S.q), worst = worstError(vecs), st = stepRad(), steps = S.q.map(t => Math.round(t / st));
  zono(p, vecs, worst);
  $('errWorst').textContent = f(worst, 2);
  $('errSw').style.background = RAMP[rampIdx(band(worst))];
  $('probeLabel').textContent = `|Δr| ≤ ${f(worst, 2)} мм`;
  bars(vecs);
  $('errQuant').textContent = f(Math.hypot(...fk(steps.map(n => n * st), S.len).map((x, i) => x - p[i])), 2);
  $('steps').textContent = steps.join(' · ');
  jacobian();
  pathLine.visible = S.trajT !== null;
  drawMap();
  drawCharts();
  render();
}

function readParams() {
  const n = id => +$('p-' + id).value;
  const len = { L1: n('L1'), L2: n('L2'), L3: n('L3') }, drive = { micro: n('micro'), backlash: n('backlash'), dL: n('dL'), z1: n('z1'), z2: n('z2') };
  const lim = { t2: [n('t2min'), n('t2max')], t3: [n('t3min'), n('t3max')] };
  if (!(len.L1 >= 0 && len.L2 > 0 && len.L3 > 0 && drive.z1 > 0 && drive.z2 > 0 && drive.backlash >= 0 && drive.dL >= 0
    && lim.t2[0] < lim.t2[1] && lim.t3[0] < lim.t3[1])) return msg('Перевірте параметри: довжини й зубці > 0, мін < макс.');
  Object.assign(S, { len, drive, lim, target: null });
  S.q[1] = clamp(S.q[1], lim.t2[0] * D, lim.t2[1] * D);
  S.q[2] = clamp(S.q[2], lim.t3[0] * D, lim.t3[1] * D);
  $('stepDeg').textContent = f(stepRad() / D, 4);
  $('dq').textContent = f(dqs()[0] / D, 3);
  msg('');
  setRanges();
  buildMap();
  buildTraj();
  update();
}
document.querySelectorAll('.param').forEach(el => el.addEventListener('change', readParams));

$('ly-map').addEventListener('change', update);
$('ly-zono').addEventListener('change', update);
$('ly-ghost').addEventListener('change', update);
$('ly-cloud').addEventListener('change', () => { paintMap(); render(); });
$('lockScale').addEventListener('change', () => { buildMap(); update(); });

function applyTheme() {
  scene.background = new THREE.Color(tok('--surface'));
  linkMat.color.set(tok('--link')); jointMat.color.set(tok('--joint')); probeMat.color.set(tok('--probe'));
  ghostMat.color.set(tok('--muted')); targetMat.color.set(tok('--warn'));
  zMesh.material.color.set(tok('--probe')); zEdges.material.color.set(tok('--probe')); pathLine.material.color.set(tok('--probe'));
  scene.remove(grid);
  grid = new THREE.GridHelper(600, 24, tok('--axis'), tok('--grid'));
  scene.add(grid);
  if (M) paintMap();
}
dark.addEventListener('change', () => { applyTheme(); update(); });

new ResizeObserver(() => {
  const { clientWidth: w, clientHeight: h } = view;
  renderer.setSize(w, h);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
  render();
}).observe(view);
new ResizeObserver(drawMap).observe($('mapwrap'));

applyTheme();
readParams();
