// Кінематика 3-ланкового маніпулятора з дисертації (розд. 1.5, 2.3).
// Кути в радіанах, довжини в мм. θ3 — відносний кут ліктя (рис. 2.8):
// абсолютний кут ланки L3 до горизонту = θ2 − θ3.

export const DEFAULTS = { L1: 120, L2: 120, L3: 70 }; // Додаток В

// Додаток В: лінійні закони руху суглобів, t ∈ [0; 1] с (рис. 2.10)
export const appendixB = t => [0.25 + 0.3 * t, 0.2 - 0.15 * t, 0.1 + 0.2 * t];

// (2.21) пряма задача
export function fk([t1, t2, t3], { L1, L2, L3 }) {
  const r = L2 * Math.cos(t2) + L3 * Math.cos(t3 - t2); // (2.23)
  const h = L2 * Math.sin(t2) - L3 * Math.sin(t3 - t2); // (2.22)
  return [r * Math.cos(t1), r * Math.sin(t1), L1 + h];
}

// (2.41) зворотна задача. elbow = +1 «лікоть вгору» (2.29), −1 «вниз» (2.30).
// У друкованій (2.41) перед γ стоїть «−», але з рис. 2.11 і (2.40) θ2 = atan2(h, r) + γ —
// саме так розв'язок збігається з (2.21) (перевірка в kin.test.mjs).
export function ik([X, Y, Z], { L1, L2, L3 }, elbow = 1) {
  const r = Math.hypot(X, Y), h = Z - L1;
  const c3 = (r * r + h * h - L2 * L2 - L3 * L3) / (2 * L2 * L3); // (2.28)
  if (c3 < -1 - 1e-9 || c3 > 1 + 1e-9) return null; // поза робочою зоною
  const s3 = elbow * Math.sqrt(Math.max(0, 1 - c3 * c3));
  const t3 = Math.atan2(s3, c3);                                   // (2.31)
  const t2 = Math.atan2(h, r) + Math.atan2(L3 * s3, L2 + L3 * c3); // (2.40), γ з (2.35)
  return [Math.atan2(Y, X), t2, t3];
}

// Стовпці якобіана ∂p/∂θj = ej × ρj (вираз 1.4) + орти ланок для похибок довжин ΔS.
export function sources([t1, t2, t3], { L2, L3 }) {
  const c1 = Math.cos(t1), s1 = Math.sin(t1), a = t2 - t3;
  const r = L2 * Math.cos(t2) + L3 * Math.cos(a);
  const h = L2 * Math.sin(t2) + L3 * Math.sin(a);
  return {
    J: [
      [-r * s1, r * c1, 0],
      [-h * c1, -h * s1, r],
      [L3 * Math.sin(a) * c1, L3 * Math.sin(a) * s1, -L3 * Math.cos(a)],
    ],
    links: [
      [0, 0, 1],
      [Math.cos(t2) * c1, Math.cos(t2) * s1, Math.sin(t2)],
      [Math.cos(a) * c1, Math.cos(a) * s1, Math.sin(a)],
    ],
  };
}

// Вектори похибки від кожного джерела: Δr = Σ Δφj (ej×ρj) + Σ ΔSi ei  (1.4)
export function errorVectors(q, len, dq, dL) {
  const { J, links } = sources(q, len);
  const v = J.map((c, j) => c.map(x => x * dq[j]));
  if (dL) links.forEach(e => v.push(e.map(x => x * dL)));
  return v;
}

// Найгірший випадок |Δr| для незалежних похибок ±δ: максимум досягається
// в одній з 2^n вершин (|·| опукла), тож просто перебираємо знаки.
export function worstError(vecs) {
  let best = 0;
  for (let m = 0; m < 1 << vecs.length; m++) {
    let x = 0, y = 0, z = 0;
    for (let i = 0; i < vecs.length; i++) {
      const s = m >> i & 1 ? 1 : -1, v = vecs[i];
      x += s * v[0]; y += s * v[1]; z += s * v[2];
    }
    best = Math.max(best, x * x + y * y + z * z);
  }
  return Math.sqrt(best);
}

// Похибка кута суглоба: ±половина мікрокроку NEMA17 після редуктора 9/32 + люфт.
export function jointError({ stepDeg = 1.8, micro = 1, gear = 9 / 32, backlashDeg = 0 }) {
  return ((stepDeg / micro) * gear / 2 + backlashDeg) * Math.PI / 180;
}
