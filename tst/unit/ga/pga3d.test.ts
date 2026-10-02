import { describe, expect, it } from 'vitest';
import type { PGA3D } from '../../../src';
import { mat4, quat, quat2, vec3 } from '../../../src';
import { pga3d } from '../../../src/ga';

// a screw motion: rotate around and translate along a tilted axis
function screwMotor(): PGA3D {
    const rotation = quat.setAxisAngle(quat.create(), vec3.normalize(vec3.create(), [1, 2, 3]), 0.7);
    const dq = quat2.fromRotationTranslation(quat2.create(), rotation, [1, -2, 0.5]);
    return pga3d.fromQuat2(pga3d.create(), dq);
}

function expectClose(actual: PGA3D, expected: PGA3D): void {
    for (let i = 0; i < 16; i++) {
        expect(actual[i]).toBeCloseTo(expected[i], 10);
    }
}

describe('pga3d', () => {
    describe('create', () => {
        it('creates a zero multivector with 16 components', () => {
            expect(pga3d.create()).toEqual(new Array(16).fill(0));
        });
    });

    describe('identity', () => {
        it('sets the scalar to 1 and everything else to 0', () => {
            const m = pga3d.identity(pga3d.point(pga3d.create(), 1, 2, 3));
            expect(m[0]).toBe(1);
            expect(m.slice(1)).toEqual(new Array(15).fill(0));
        });
    });

    describe('point and toVec3', () => {
        it('stores coordinates in e032, e013, e021 with weight 1 in e123', () => {
            const p = pga3d.point(pga3d.create(), 1, 2, 3);
            expect(p[13]).toBe(1);
            expect(p[12]).toBe(2);
            expect(p[11]).toBe(3);
            expect(p[14]).toBe(1);
        });

        it('divides out the homogeneous weight', () => {
            const p = pga3d.scale(pga3d.create(), pga3d.point(pga3d.create(), 1, 2, 3), 4);
            expect(pga3d.toVec3(vec3.create(), p)).toEqual([1, 2, 3]);
        });
    });

    describe('plane', () => {
        it('stores the normal in e1, e2, e3 and the distance term in e0', () => {
            const p = pga3d.plane(pga3d.create(), 1, 2, 3, 4);
            expect(p[1]).toBe(4);
            expect(p[2]).toBe(1);
            expect(p[3]).toBe(2);
            expect(p[4]).toBe(3);
        });
    });

    describe('multiply', () => {
        it('follows the metric: e1 squares to 1, e0 squares to 0, e12 squares to -1', () => {
            const e0: PGA3D = [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
            const e1: PGA3D = [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
            const e12: PGA3D = [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0];
            const out = pga3d.create();
            expect(pga3d.multiply(out, e1, e1)[0]).toBe(1);
            expect(pga3d.multiply(out, e0, e0)).toEqual(pga3d.create());
            expect(pga3d.multiply(out, e12, e12)[0]).toBe(-1);
        });

        it('composes motors like quat2.multiply', () => {
            const a = quat2.fromRotationTranslation(quat2.create(), quat.setAxisAngle(quat.create(), [0, 0, 1], 0.5), [1, 2, 3]);
            const b = quat2.fromRotationTranslation(
                quat2.create(),
                quat.setAxisAngle(quat.create(), [1, 0, 0], -0.3),
                [-2, 0, 1],
            );
            const ma = pga3d.fromQuat2(pga3d.create(), a);
            const mb = pga3d.fromQuat2(pga3d.create(), b);
            const product = pga3d.toQuat2(quat2.create(), pga3d.multiply(pga3d.create(), ma, mb));
            const expected = quat2.multiply(quat2.create(), a, b);
            for (let i = 0; i < 8; i++) {
                expect(product[i]).toBeCloseTo(expected[i], 10);
            }
        });

        it('is safe when out aliases an input', () => {
            const a = screwMotor();
            const b = pga3d.point(pga3d.create(), 1, 2, 3);
            const expected = pga3d.multiply(pga3d.create(), a, b);
            expectClose(pga3d.multiply(a, a, b), expected);
        });
    });

    describe('sandwich', () => {
        it('transforms a point like the equivalent dual quaternion matrix', () => {
            const m = screwMotor();
            const p = pga3d.point(pga3d.create(), 0.5, -1, 2);
            const moved = pga3d.toVec3(vec3.create(), pga3d.sandwich(pga3d.create(), m, p));

            const matrix = mat4.fromQuat2(mat4.create(), pga3d.toQuat2(quat2.create(), m));
            const expected = vec3.transformMat4(vec3.create(), [0.5, -1, 2], matrix);
            expect(moved[0]).toBeCloseTo(expected[0], 10);
            expect(moved[1]).toBeCloseTo(expected[1], 10);
            expect(moved[2]).toBeCloseTo(expected[2], 10);
        });

        it('keeps a point on a plane when both are moved by the same motor', () => {
            const m = screwMotor();
            const plane = pga3d.plane(pga3d.create(), 0, 0, 1, -2);
            const p = pga3d.point(pga3d.create(), 3, 4, 2);
            const movedPlane = pga3d.sandwich(pga3d.create(), m, plane);
            const movedPoint = pga3d.sandwich(pga3d.create(), m, p);
            // a point lies on a plane when their outer product vanishes
            const incidence = pga3d.wedge(pga3d.create(), movedPlane, movedPoint);
            expect(pga3d.idealNorm(incidence)).toBeCloseTo(0, 10);
        });
    });

    describe('fromTranslation', () => {
        it('translates a point', () => {
            const t = pga3d.fromTranslation(pga3d.create(), [1, 2, 3]);
            const p = pga3d.sandwich(pga3d.create(), t, pga3d.point(pga3d.create(), 0.5, -1, 2));
            const result = pga3d.toVec3(vec3.create(), p);
            expect(result[0]).toBeCloseTo(1.5, 10);
            expect(result[1]).toBeCloseTo(1, 10);
            expect(result[2]).toBeCloseTo(5, 10);
        });
    });

    describe('fromAxisAngle', () => {
        it('rotates (1, 0, 0) to (0, 1, 0) around z by a quarter turn', () => {
            const r = pga3d.fromAxisAngle(pga3d.create(), [0, 0, 1], Math.PI / 2);
            const p = pga3d.sandwich(pga3d.create(), r, pga3d.point(pga3d.create(), 1, 0, 0));
            const result = pga3d.toVec3(vec3.create(), p);
            expect(result[0]).toBeCloseTo(0, 10);
            expect(result[1]).toBeCloseTo(1, 10);
            expect(result[2]).toBeCloseTo(0, 10);
        });

        it('matches quat.setAxisAngle through toQuat2', () => {
            const axis = vec3.normalize(vec3.create(), [1, 2, 3]);
            const r = pga3d.fromAxisAngle(pga3d.create(), axis, 0.7);
            const q = pga3d.toQuat2(quat2.create(), r);
            const expected = quat.setAxisAngle(quat.create(), axis, 0.7);
            for (let i = 0; i < 4; i++) {
                expect(q[i]).toBeCloseTo(expected[i], 10);
            }
            for (let i = 4; i < 8; i++) {
                expect(q[i]).toBeCloseTo(0, 10);
            }
        });
    });

    describe('fromQuat2 and toQuat2', () => {
        it('round trip', () => {
            const dq = quat2.fromRotationTranslation(quat2.create(), quat.setAxisAngle(quat.create(), [0, 1, 0], 1.2), [1, 2, 3]);
            const result = pga3d.toQuat2(quat2.create(), pga3d.fromQuat2(pga3d.create(), dq));
            expect(result).toEqual(dq);
        });
    });

    describe('wedge', () => {
        it('meets two planes in their intersection line and the line with a plane in a point', () => {
            const z0 = pga3d.plane(pga3d.create(), 0, 0, 1, 0);
            const x1 = pga3d.plane(pga3d.create(), 1, 0, 0, -1);
            const y2 = pga3d.plane(pga3d.create(), 0, 1, 0, -2);
            const line = pga3d.wedge(pga3d.create(), z0, x1);
            // the line x = 1, z = 0 runs along y
            expect(line[9]).toBe(1);
            expect(pga3d.norm(line)).toBeCloseTo(1, 10);
            const p = pga3d.wedge(pga3d.create(), line, y2);
            const result = pga3d.toVec3(vec3.create(), p);
            expect(result[0]).toBeCloseTo(1, 10);
            expect(result[1]).toBeCloseTo(2, 10);
            expect(result[2]).toBeCloseTo(0, 10);
        });

        it('vanishes for parallel planes', () => {
            const a = pga3d.plane(pga3d.create(), 0, 0, 1, 0);
            const b = pga3d.plane(pga3d.create(), 0, 0, 1, -3);
            const line = pga3d.wedge(pga3d.create(), a, b);
            expect(pga3d.norm(line)).toBe(0);
            // parallel planes meet in an ideal line, the direction at infinity
            expect(pga3d.idealNorm(line)).toBeCloseTo(3, 10);
        });
    });

    describe('vee', () => {
        it('joins two points into the line from the first to the second', () => {
            const origin = pga3d.point(pga3d.create(), 0, 0, 0);
            const x = pga3d.point(pga3d.create(), 1, 0, 0);
            const line = pga3d.vee(pga3d.create(), origin, x);
            // direction +x is e23, the line passes through the origin so it has no moment
            expect(line[10]).toBe(1);
            expect(pga3d.idealNorm(line)).toBe(0);
        });

        it('joins a line and a point into the plane through them', () => {
            const a = pga3d.point(pga3d.create(), 0, 0, 0);
            const b = pga3d.point(pga3d.create(), 1, 0, 0);
            const c = pga3d.point(pga3d.create(), 0, 1, 0);
            const line = pga3d.vee(pga3d.create(), a, b);
            const plane = pga3d.normalize(pga3d.create(), pga3d.vee(pga3d.create(), line, c));
            // the xy plane z = 0, up to orientation
            expect(Math.abs(plane[4])).toBeCloseTo(1, 10);
            expect(plane[1]).toBeCloseTo(0, 10);
            expect(plane[2]).toBeCloseTo(0, 10);
            expect(plane[3]).toBeCloseTo(0, 10);
        });
    });

    describe('dot', () => {
        it('gives the cosine of the angle between two normalized planes', () => {
            const a = pga3d.plane(pga3d.create(), 1, 0, 0, 5);
            const b = pga3d.plane(pga3d.create(), Math.SQRT1_2, Math.SQRT1_2, 0, -1);
            const result = pga3d.dot(pga3d.create(), a, b);
            expect(result[0]).toBeCloseTo(Math.SQRT1_2, 10);
            expect(result.slice(1)).toEqual(new Array(15).fill(0));
        });
    });

    describe('involutions', () => {
        it('reverse flips grades 2 and 3, involute flips odd grades, conjugate flips grades 1 and 2', () => {
            const a: PGA3D = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16];
            expect(pga3d.reverse(pga3d.create(), a)).toEqual([1, 2, 3, 4, 5, -6, -7, -8, -9, -10, -11, -12, -13, -14, -15, 16]);
            expect(pga3d.involute(pga3d.create(), a)).toEqual([1, -2, -3, -4, -5, 6, 7, 8, 9, 10, 11, -12, -13, -14, -15, 16]);
            expect(pga3d.conjugate(pga3d.create(), a)).toEqual([1, -2, -3, -4, -5, -6, -7, -8, -9, -10, -11, 12, 13, 14, 15, 16]);
        });
    });

    describe('dual and undual', () => {
        it('maps the origin to the plane at infinity and undoes itself', () => {
            const origin = pga3d.point(pga3d.create(), 0, 0, 0);
            const d = pga3d.dual(pga3d.create(), origin);
            expect(d[1]).toBe(-1);
            expect(pga3d.norm(d)).toBe(0);
            const a: PGA3D = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16];
            expect(pga3d.undual(pga3d.create(), pga3d.dual(pga3d.create(), a))).toEqual(a);
        });
    });

    describe('norm and idealNorm', () => {
        it('measures the weight of a point and the normal length of a plane', () => {
            const p = pga3d.scale(pga3d.create(), pga3d.point(pga3d.create(), 1, 2, 3), -2);
            expect(pga3d.norm(p)).toBe(2);
            const plane = pga3d.plane(pga3d.create(), 3, 0, 4, 7);
            expect(pga3d.norm(plane)).toBe(5);
            expect(pga3d.idealNorm(plane)).toBe(7);
        });
    });

    describe('normalize', () => {
        it('restores a drifted motor so that m * reverse(m) = 1', () => {
            const drifted = pga3d.scale(pga3d.create(), screwMotor(), 1.7);
            drifted[15] += 0.3;
            const m = pga3d.normalize(pga3d.create(), drifted);
            const check = pga3d.multiply(pga3d.create(), m, pga3d.reverse(pga3d.create(), m));
            expectClose(check, pga3d.identity(pga3d.create()));
        });

        it('leaves a zero multivector untouched', () => {
            expect(pga3d.normalize(pga3d.create(), pga3d.create())).toEqual(pga3d.create());
        });
    });

    describe('invert', () => {
        it('inverts a scaled motor and a plane', () => {
            const m = pga3d.scale(pga3d.create(), screwMotor(), 1.7);
            m[15] += 0.3;
            const check = pga3d.multiply(pga3d.create(), pga3d.invert(pga3d.create(), m), m);
            expectClose(check, pga3d.identity(pga3d.create()));

            const plane = pga3d.plane(pga3d.create(), 3, 0, 4, 7);
            const planeCheck = pga3d.multiply(pga3d.create(), pga3d.invert(pga3d.create(), plane), plane);
            expectClose(planeCheck, pga3d.identity(pga3d.create()));
        });

        it('inverts an odd element whose norm has an e0123 part from both sides', () => {
            const drifted = pga3d.scale(pga3d.create(), screwMotor(), 1.3);
            drifted[15] += 0.2;
            const a = pga3d.multiply(pga3d.create(), pga3d.plane(pga3d.create(), 3, 0, 4, 7), drifted);
            const inverse = pga3d.invert(pga3d.create(), a);
            const left = pga3d.multiply(pga3d.create(), inverse, a);
            const right = pga3d.multiply(pga3d.create(), a, inverse);
            for (let i = 0; i < 16; i++) {
                expect(left[i]).toBeCloseTo(i === 0 ? 1 : 0, 12);
                expect(right[i]).toBeCloseTo(i === 0 ? 1 : 0, 12);
            }
        });
    });

    describe('exp and log', () => {
        it('exp of a scaled line rotates around it', () => {
            const line = pga3d.vee(pga3d.create(), pga3d.point(pga3d.create(), 0, 0, 0), pga3d.point(pga3d.create(), 0, 0, 1));
            const r = pga3d.exp(pga3d.create(), pga3d.scale(pga3d.create(), line, -Math.PI / 4));
            const p = pga3d.sandwich(pga3d.create(), r, pga3d.point(pga3d.create(), 1, 0, 0));
            const result = pga3d.toVec3(vec3.create(), p);
            expect(result[0]).toBeCloseTo(0, 10);
            expect(result[1]).toBeCloseTo(1, 10);
            expect(result[2]).toBeCloseTo(0, 10);
        });

        it('exp of an ideal line translates', () => {
            const t = pga3d.exp(pga3d.create(), [0, 0, 0, 0, 0, -0.5, -1, -1.5, 0, 0, 0, 0, 0, 0, 0, 0]);
            expectClose(t, pga3d.fromTranslation(pga3d.create(), [1, 2, 3]));
        });

        it('round trips a screw motor through log and exp', () => {
            const m = screwMotor();
            const b = pga3d.log(pga3d.create(), m);
            expectClose(pga3d.exp(pga3d.create(), b), m);
        });

        it('log of a translator is its ideal part', () => {
            const t = pga3d.fromTranslation(pga3d.create(), [1, 2, 3]);
            expect(pga3d.log(pga3d.create(), t)).toEqual([0, 0, 0, 0, 0, -0.5, -1, -1.5, 0, 0, 0, 0, 0, 0, 0, 0]);
        });

        it('log of a full turn translator is the same translation', () => {
            const t = pga3d.scale(pga3d.create(), pga3d.fromTranslation(pga3d.create(), [1, 2, 3]), -1);
            expect(pga3d.log(pga3d.create(), t)).toEqual([0, 0, 0, 0, 0, -0.5, -1, -1.5, 0, 0, 0, 0, 0, 0, 0, 0]);
        });

        it('exp of a screw along z matches the closed form at tiny and moderate angles', () => {
            // exp(h e12 + d e03) = cos h + sin h e12 + d cos h e03 + d sin h e0123
            const d = 0.75;
            for (const h of [1e-8, 1e-4, 0.03, 0.04, 0.5]) {
                const m = pga3d.exp(pga3d.create(), [0, 0, 0, 0, 0, 0, 0, d, h, 0, 0, 0, 0, 0, 0, 0]);
                const expected: PGA3D = [
                    Math.cos(h),
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    d * Math.cos(h),
                    Math.sin(h),
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    d * Math.sin(h),
                ];
                for (let i = 0; i < 16; i++) {
                    expect(m[i]).toBeCloseTo(expected[i], 15);
                }
            }
        });

        it('log recovers a screw with a 1e-8 rad rotation', () => {
            // half angle 5e-9 around the tilted line through (1, -2, 0.5), with a translation along it
            const axis = vec3.normalize(vec3.create(), [1, 2, 3]);
            const moment = vec3.cross(vec3.create(), [1, -2, 0.5], axis);
            const h = 5e-9;
            const b: PGA3D = [0, 0, 0, 0, 0, 0, 0, 0, h * axis[2], h * axis[1], h * axis[0], 0, 0, 0, 0, 0];
            b[5] = h * moment[0] + 0.4 * axis[0];
            b[6] = h * moment[1] + 0.4 * axis[1];
            b[7] = h * moment[2] + 0.4 * axis[2];
            const m = pga3d.exp(pga3d.create(), b);
            const result = pga3d.log(pga3d.create(), m);
            for (const i of [8, 9, 10]) {
                expect(result[i] / b[i]).toBeCloseTo(1, 12);
            }
            for (let i = 0; i < 16; i++) {
                expect(pga3d.exp(pga3d.create(), result)[i]).toBeCloseTo(m[i], 12);
            }
        });
    });

    describe('grade', () => {
        it('keeps a single grade', () => {
            const a: PGA3D = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16];
            expect(pga3d.grade(pga3d.create(), a, 2)).toEqual([0, 0, 0, 0, 0, 6, 7, 8, 9, 10, 11, 0, 0, 0, 0, 0]);
            expect(pga3d.grade(pga3d.create(), a, 4)).toEqual([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 16]);
        });
    });

    describe('str', () => {
        it('lists the non zero blades', () => {
            expect(pga3d.str(pga3d.point(pga3d.create(), 1, 2, 3))).toBe('pga3d(3e021 + 2e013 + 1e032 + 1e123)');
            expect(pga3d.str(pga3d.create())).toBe('pga3d(0)');
        });
    });

    describe('equals', () => {
        it('compares approximately and exactly', () => {
            const a = screwMotor();
            const b = pga3d.clone(a);
            b[0] += 1e-9;
            expect(pga3d.equals(a, b)).toBe(true);
            expect(pga3d.exactEquals(a, b)).toBe(false);
            expect(pga3d.exactEquals(a, pga3d.copy(pga3d.create(), a))).toBe(true);
        });
    });
});
