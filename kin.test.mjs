// node kin.test.mjs
import assert from 'node:assert/strict';
import { DEFAULTS as L, fk, ik, sources, worstError, jointError, appendixB } from './kin.js';

const close = (a, b, eps = 1e-6) => a.forEach((x, i) => assert.ok(Math.abs(x - b[i]) < eps, `${a} != ${b}`));

// IK(FK(q)) == q для обох конфігурацій ліктя
for (const q of [[0.4, 0.6, 0.9], [-2, 1.2, 0.3], [1, -0.3, -1.1], [3, 0.1, -2.5]]) {
  close(ik(fk(q, L), L, Math.sign(q[2])), q);
  close(fk(ik(fk(q, L), L, -Math.sign(q[2])), L), fk(q, L)); // інша конфігурація — та сама точка
}
assert.equal(ik([500, 0, 120], L), null);

// аналітичний якобіан == скінченні різниці
const q = [0.7, 0.5, 1.3], e = 1e-6;
sources(q, L).J.forEach((col, j) => {
  const qp = [...q]; qp[j] += e;
  close(col, fk(qp, L).map((x, i) => (x - fk(q, L)[i]) / e), 1e-3);
});

// перебір знаків: ортогональні вектори → довжина діагоналі
assert.ok(Math.abs(worstError([[3, 0, 0], [0, 4, 0]]) - 5) < 1e-12);
// повний крок NEMA17 через 9/32: ±0.253°
assert.ok(Math.abs(jointError({}) * 180 / Math.PI - 0.253125) < 1e-9);

// траєкторія Додатку В лягає в діапазони осей рис. 2.10 (X 160…180, Y 40…100, Z 100…150 мм)
close(fk(appendixB(0), L), [181.4, 46.3, 150.8], 0.05);
close(fk(appendixB(1), L), [160.0, 98.1, 108.7], 0.05);

console.log('ok');
