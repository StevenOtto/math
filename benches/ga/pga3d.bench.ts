import { bench, group } from "@pmndrs/labs";
import * as quat from "../../src/core/quat";
import * as quat2 from "../../src/core/quat2";
import * as pga3d from "../../src/ga/pga3d";
import type { PGA3D } from "../../src/ga/pga3d";
import * as mulberry32 from "../../src/random/mulberry32";

const N = 10_000;

function makeMotors(seed: number): PGA3D[] {
  const rand = mulberry32.create(seed);
  const out: PGA3D[] = [];
  for (let i = 0; i < N; i++) {
    const q = quat.setAxisAngle(
      quat.create(),
      [0.267261, 0.534522, 0.801784],
      mulberry32.sample(rand) * Math.PI * 2,
    );
    const dq = quat2.fromRotationTranslation(quat2.create(), q, [
      mulberry32.sample(rand) * 10,
      mulberry32.sample(rand) * 10,
      mulberry32.sample(rand) * 10,
    ]);
    out.push(pga3d.fromQuat2(pga3d.create(), dq));
  }
  return out;
}

function makePoints(seed: number): PGA3D[] {
  const rand = mulberry32.create(seed);
  const out: PGA3D[] = [];
  for (let i = 0; i < N; i++) {
    out.push(
      pga3d.point(
        pga3d.create(),
        mulberry32.sample(rand) * 10,
        mulberry32.sample(rand) * 10,
        mulberry32.sample(rand) * 10,
      ),
    );
  }
  return out;
}

group("pga3d ops 10k @ga @pga3d", () => {
  bench("multiply", function* () {
    const a = makeMotors(1);
    const b = makeMotors(2);
    const out = pga3d.create();

    const acc = yield () => {
      let acc = 0;
      for (let i = 0; i < N; i++) {
        pga3d.multiply(out, a[i], b[i]);
        acc += out[0];
      }
      return acc;
    };
    return [acc, ...out];
  });

  bench("sandwich point", function* () {
    const m = makeMotors(1);
    const p = makePoints(2);
    const out = pga3d.create();

    const acc = yield () => {
      let acc = 0;
      for (let i = 0; i < N; i++) {
        pga3d.sandwich(out, m[i], p[i]);
        acc += out[13];
      }
      return acc;
    };
    return [acc, ...out];
  });

  bench("normalize", function* () {
    const m = makeMotors(1);
    const out = pga3d.create();

    const acc = yield () => {
      let acc = 0;
      for (let i = 0; i < N; i++) {
        pga3d.normalize(out, m[i]);
        acc += out[0];
      }
      return acc;
    };
    return [acc, ...out];
  });

  bench("exp(log)", function* () {
    const m = makeMotors(1);
    const b = pga3d.create();
    const out = pga3d.create();

    const acc = yield () => {
      let acc = 0;
      for (let i = 0; i < N; i++) {
        pga3d.exp(out, pga3d.log(b, m[i]));
        acc += out[0];
      }
      return acc;
    };
    return [acc, ...out];
  });
});
