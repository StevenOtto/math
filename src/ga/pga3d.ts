import type { Const } from '../core/const';
import type { Quat2 } from '../core/quat2';
import { EPSILON } from '../core/scalar';
import type { Vec3 } from '../core/vec3';

/**
 * A multivector of 3D projective geometric algebra, Cl(3,0,1), where e0 squares to 0 and e1, e2, e3 square to 1.
 *
 * Components are stored in the bivector.net order:
 * [1, e0, e1, e2, e3, e01, e02, e03, e12, e31, e23, e021, e013, e032, e123, e0123]
 *
 * A plane ax + by + cz + d = 0 is the vector a e1 + b e2 + c e3 + d e0.
 * A line is a bivector with direction in e23, e31, e12 and moment in e01, e02, e03.
 * A point (x, y, z) is the trivector x e032 + y e013 + z e021 + e123.
 * A motor (rotation and translation) is a normalized even multivector, the PGA counterpart of a Quat2.
 */
export type PGA3D = [
    s: number,
    e0: number,
    e1: number,
    e2: number,
    e3: number,
    e01: number,
    e02: number,
    e03: number,
    e12: number,
    e31: number,
    e23: number,
    e021: number,
    e013: number,
    e032: number,
    e123: number,
    e0123: number,
];

/**
 * Creates a new zero multivector
 *
 * @returns a new multivector
 */
export function create(): PGA3D {
    return [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
}

/**
 * Creates a new multivector initialized with values from an existing multivector
 *
 * @param a multivector to clone
 * @returns a new multivector
 */
export function clone(a: Const<PGA3D>): PGA3D {
    return [a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], a[8], a[9], a[10], a[11], a[12], a[13], a[14], a[15]];
}

/**
 * Copy the values from one multivector to another
 *
 * @param out the receiving multivector
 * @param a the source multivector
 * @returns out
 */
export function copy(out: PGA3D, a: Const<PGA3D>): PGA3D {
    for (let i = 0; i < 16; i++) {
        out[i] = a[i];
    }
    return out;
}

/**
 * Set a multivector to the scalar 1, the identity motor
 *
 * @param out the receiving multivector
 * @returns out
 */
export function identity(out: PGA3D): PGA3D {
    out[0] = 1;
    for (let i = 1; i < 16; i++) {
        out[i] = 0;
    }
    return out;
}

/**
 * Sets a multivector to the plane ax + by + cz + d = 0
 *
 * @param out the receiving multivector
 * @param a X coefficient of the plane normal
 * @param b Y coefficient of the plane normal
 * @param c Z coefficient of the plane normal
 * @param d signed distance term
 * @returns out
 */
export function plane(out: PGA3D, a: number, b: number, c: number, d: number): PGA3D {
    for (let i = 0; i < 16; i++) {
        out[i] = 0;
    }
    out[1] = d;
    out[2] = a;
    out[3] = b;
    out[4] = c;
    return out;
}

/**
 * Sets a multivector to the normalized point (x, y, z)
 *
 * @param out the receiving multivector
 * @param x X coordinate
 * @param y Y coordinate
 * @param z Z coordinate
 * @returns out
 */
export function point(out: PGA3D, x: number, y: number, z: number): PGA3D {
    for (let i = 0; i < 16; i++) {
        out[i] = 0;
    }
    out[11] = z;
    out[12] = y;
    out[13] = x;
    out[14] = 1;
    return out;
}

/**
 * Reads the euclidean coordinates of a point, dividing out the homogeneous e123 weight
 *
 * @param out the receiving vector
 * @param p a point multivector with non zero weight
 * @returns out
 */
export function toVec3(out: Vec3, p: Const<PGA3D>): Vec3 {
    const w = 1 / p[14];
    out[0] = p[13] * w;
    out[1] = p[12] * w;
    out[2] = p[11] * w;
    return out;
}

/**
 * Sets a multivector to the motor that translates by the given vector
 *
 * @param out the receiving multivector
 * @param v translation vector
 * @returns out
 */
export function fromTranslation(out: PGA3D, v: Const<Vec3>): PGA3D {
    identity(out);
    out[5] = -0.5 * v[0];
    out[6] = -0.5 * v[1];
    out[7] = -0.5 * v[2];
    return out;
}

/**
 * Sets a multivector to the motor that rotates around an axis through the origin.
 * Matches quat.setAxisAngle, so fromAxisAngle followed by toQuat2 gives the same quaternion.
 *
 * @param out the receiving multivector
 * @param axis the axis to rotate around, normalized
 * @param rad the angle in radians
 * @returns out
 */
export function fromAxisAngle(out: PGA3D, axis: Const<Vec3>, rad: number): PGA3D {
    const s = -Math.sin(rad * 0.5);
    identity(out);
    out[0] = Math.cos(rad * 0.5);
    out[8] = s * axis[2];
    out[9] = s * axis[1];
    out[10] = s * axis[0];
    return out;
}

/**
 * Sets a multivector to the motor equivalent to a dual quaternion
 *
 * @param out the receiving multivector
 * @param q a dual quaternion
 * @returns out
 */
export function fromQuat2(out: PGA3D, q: Const<Quat2>): PGA3D {
    out[0] = q[3];
    out[1] = 0;
    out[2] = 0;
    out[3] = 0;
    out[4] = 0;
    out[5] = -q[4];
    out[6] = -q[5];
    out[7] = -q[6];
    out[8] = -q[2];
    out[9] = -q[1];
    out[10] = -q[0];
    out[11] = 0;
    out[12] = 0;
    out[13] = 0;
    out[14] = 0;
    out[15] = -q[7];
    return out;
}

/**
 * Converts the even part of a multivector to a dual quaternion
 *
 * @param out the receiving dual quaternion
 * @param m a motor
 * @returns out
 */
export function toQuat2(out: Quat2, m: Const<PGA3D>): Quat2 {
    out[0] = -m[10];
    out[1] = -m[9];
    out[2] = -m[8];
    out[3] = m[0];
    out[4] = -m[5];
    out[5] = -m[6];
    out[6] = -m[7];
    out[7] = -m[15];
    return out;
}

/**
 * Adds two multivectors
 *
 * @param out the receiving multivector
 * @param a the first operand
 * @param b the second operand
 * @returns out
 */
export function add(out: PGA3D, a: Const<PGA3D>, b: Const<PGA3D>): PGA3D {
    for (let i = 0; i < 16; i++) {
        out[i] = a[i] + b[i];
    }
    return out;
}

/**
 * Subtracts multivector b from multivector a
 *
 * @param out the receiving multivector
 * @param a the first operand
 * @param b the second operand
 * @returns out
 */
export function subtract(out: PGA3D, a: Const<PGA3D>, b: Const<PGA3D>): PGA3D {
    for (let i = 0; i < 16; i++) {
        out[i] = a[i] - b[i];
    }
    return out;
}

/**
 * Alias for {@link pga3d.subtract}
 * @function
 */
export const sub = subtract;

/**
 * Scales a multivector by a scalar number
 *
 * @param out the receiving multivector
 * @param a the multivector to scale
 * @param b amount to scale the multivector by
 * @returns out
 */
export function scale(out: PGA3D, a: Const<PGA3D>, b: number): PGA3D {
    for (let i = 0; i < 16; i++) {
        out[i] = a[i] * b;
    }
    return out;
}

/**
 * Keeps the components of a single grade and zeroes the rest
 *
 * @param out the receiving multivector
 * @param a the source multivector
 * @param k the grade to keep, 0 to 4
 * @returns out
 */
export function grade(out: PGA3D, a: Const<PGA3D>, k: number): PGA3D {
    // component index ranges per grade in the bivector.net order
    const start = [0, 1, 5, 11, 15][k];
    const end = [1, 5, 11, 15, 16][k];
    for (let i = 0; i < 16; i++) {
        out[i] = i >= start && i < end ? a[i] : 0;
    }
    return out;
}

/**
 * Geometric product of two multivectors. Composes motors: multiply(out, b, a) applies a then b.
 *
 * @param out the receiving multivector
 * @param a the first operand
 * @param b the second operand
 * @returns out
 */
export function multiply(out: PGA3D, a: Const<PGA3D>, b: Const<PGA3D>): PGA3D {
    const a0 = a[0];
    const a1 = a[1];
    const a2 = a[2];
    const a3 = a[3];
    const a4 = a[4];
    const a5 = a[5];
    const a6 = a[6];
    const a7 = a[7];
    const a8 = a[8];
    const a9 = a[9];
    const a10 = a[10];
    const a11 = a[11];
    const a12 = a[12];
    const a13 = a[13];
    const a14 = a[14];
    const a15 = a[15];
    const b0 = b[0];
    const b1 = b[1];
    const b2 = b[2];
    const b3 = b[3];
    const b4 = b[4];
    const b5 = b[5];
    const b6 = b[6];
    const b7 = b[7];
    const b8 = b[8];
    const b9 = b[9];
    const b10 = b[10];
    const b11 = b[11];
    const b12 = b[12];
    const b13 = b[13];
    const b14 = b[14];
    const b15 = b[15];
    out[0] = a0 * b0 + a2 * b2 + a3 * b3 + a4 * b4 - a8 * b8 - a9 * b9 - a10 * b10 - a14 * b14;
    out[1] =
        a0 * b1 +
        a1 * b0 -
        a2 * b5 -
        a3 * b6 -
        a4 * b7 +
        a5 * b2 +
        a6 * b3 +
        a7 * b4 +
        a8 * b11 +
        a9 * b12 +
        a10 * b13 +
        a11 * b8 +
        a12 * b9 +
        a13 * b10 +
        a14 * b15 -
        a15 * b14;
    out[2] = a0 * b2 + a2 * b0 - a3 * b8 + a4 * b9 + a8 * b3 - a9 * b4 - a10 * b14 - a14 * b10;
    out[3] = a0 * b3 + a2 * b8 + a3 * b0 - a4 * b10 - a8 * b2 - a9 * b14 + a10 * b4 - a14 * b9;
    out[4] = a0 * b4 - a2 * b9 + a3 * b10 + a4 * b0 - a8 * b14 + a9 * b2 - a10 * b3 - a14 * b8;
    out[5] =
        a0 * b5 +
        a1 * b2 -
        a2 * b1 -
        a3 * b11 +
        a4 * b12 +
        a5 * b0 -
        a6 * b8 +
        a7 * b9 +
        a8 * b6 -
        a9 * b7 -
        a10 * b15 -
        a11 * b3 +
        a12 * b4 +
        a13 * b14 -
        a14 * b13 -
        a15 * b10;
    out[6] =
        a0 * b6 +
        a1 * b3 +
        a2 * b11 -
        a3 * b1 -
        a4 * b13 +
        a5 * b8 +
        a6 * b0 -
        a7 * b10 -
        a8 * b5 -
        a9 * b15 +
        a10 * b7 +
        a11 * b2 +
        a12 * b14 -
        a13 * b4 -
        a14 * b12 -
        a15 * b9;
    out[7] =
        a0 * b7 +
        a1 * b4 -
        a2 * b12 +
        a3 * b13 -
        a4 * b1 -
        a5 * b9 +
        a6 * b10 +
        a7 * b0 -
        a8 * b15 +
        a9 * b5 -
        a10 * b6 +
        a11 * b14 -
        a12 * b2 +
        a13 * b3 -
        a14 * b11 -
        a15 * b8;
    out[8] = a0 * b8 + a2 * b3 - a3 * b2 + a4 * b14 + a8 * b0 + a9 * b10 - a10 * b9 + a14 * b4;
    out[9] = a0 * b9 - a2 * b4 + a3 * b14 + a4 * b2 - a8 * b10 + a9 * b0 + a10 * b8 + a14 * b3;
    out[10] = a0 * b10 + a2 * b14 + a3 * b4 - a4 * b3 + a8 * b9 - a9 * b8 + a10 * b0 + a14 * b2;
    out[11] =
        a0 * b11 -
        a1 * b8 +
        a2 * b6 -
        a3 * b5 +
        a4 * b15 -
        a5 * b3 +
        a6 * b2 -
        a7 * b14 -
        a8 * b1 +
        a9 * b13 -
        a10 * b12 +
        a11 * b0 +
        a12 * b10 -
        a13 * b9 +
        a14 * b7 -
        a15 * b4;
    out[12] =
        a0 * b12 -
        a1 * b9 -
        a2 * b7 +
        a3 * b15 +
        a4 * b5 +
        a5 * b4 -
        a6 * b14 -
        a7 * b2 -
        a8 * b13 -
        a9 * b1 +
        a10 * b11 -
        a11 * b10 +
        a12 * b0 +
        a13 * b8 +
        a14 * b6 -
        a15 * b3;
    out[13] =
        a0 * b13 -
        a1 * b10 +
        a2 * b15 +
        a3 * b7 -
        a4 * b6 -
        a5 * b14 -
        a6 * b4 +
        a7 * b3 +
        a8 * b12 -
        a9 * b11 -
        a10 * b1 +
        a11 * b9 -
        a12 * b8 +
        a13 * b0 +
        a14 * b5 -
        a15 * b2;
    out[14] = a0 * b14 + a2 * b10 + a3 * b9 + a4 * b8 + a8 * b4 + a9 * b3 + a10 * b2 + a14 * b0;
    out[15] =
        a0 * b15 +
        a1 * b14 +
        a2 * b13 +
        a3 * b12 +
        a4 * b11 +
        a5 * b10 +
        a6 * b9 +
        a7 * b8 +
        a8 * b7 +
        a9 * b6 +
        a10 * b5 -
        a11 * b4 -
        a12 * b3 -
        a13 * b2 -
        a14 * b1 +
        a15 * b0;
    return out;
}

/**
 * Alias for {@link pga3d.multiply}
 * @function
 */
export const mul = multiply;

/**
 * Outer product of two multivectors, the meet. Two planes meet in a line, a line and a plane meet in a point.
 *
 * @param out the receiving multivector
 * @param a the first operand
 * @param b the second operand
 * @returns out
 */
export function wedge(out: PGA3D, a: Const<PGA3D>, b: Const<PGA3D>): PGA3D {
    const a0 = a[0];
    const a1 = a[1];
    const a2 = a[2];
    const a3 = a[3];
    const a4 = a[4];
    const a5 = a[5];
    const a6 = a[6];
    const a7 = a[7];
    const a8 = a[8];
    const a9 = a[9];
    const a10 = a[10];
    const a11 = a[11];
    const a12 = a[12];
    const a13 = a[13];
    const a14 = a[14];
    const a15 = a[15];
    const b0 = b[0];
    const b1 = b[1];
    const b2 = b[2];
    const b3 = b[3];
    const b4 = b[4];
    const b5 = b[5];
    const b6 = b[6];
    const b7 = b[7];
    const b8 = b[8];
    const b9 = b[9];
    const b10 = b[10];
    const b11 = b[11];
    const b12 = b[12];
    const b13 = b[13];
    const b14 = b[14];
    const b15 = b[15];
    out[0] = a0 * b0;
    out[1] = a0 * b1 + a1 * b0;
    out[2] = a0 * b2 + a2 * b0;
    out[3] = a0 * b3 + a3 * b0;
    out[4] = a0 * b4 + a4 * b0;
    out[5] = a0 * b5 + a1 * b2 - a2 * b1 + a5 * b0;
    out[6] = a0 * b6 + a1 * b3 - a3 * b1 + a6 * b0;
    out[7] = a0 * b7 + a1 * b4 - a4 * b1 + a7 * b0;
    out[8] = a0 * b8 + a2 * b3 - a3 * b2 + a8 * b0;
    out[9] = a0 * b9 - a2 * b4 + a4 * b2 + a9 * b0;
    out[10] = a0 * b10 + a3 * b4 - a4 * b3 + a10 * b0;
    out[11] = a0 * b11 - a1 * b8 + a2 * b6 - a3 * b5 - a5 * b3 + a6 * b2 - a8 * b1 + a11 * b0;
    out[12] = a0 * b12 - a1 * b9 - a2 * b7 + a4 * b5 + a5 * b4 - a7 * b2 - a9 * b1 + a12 * b0;
    out[13] = a0 * b13 - a1 * b10 + a3 * b7 - a4 * b6 - a6 * b4 + a7 * b3 - a10 * b1 + a13 * b0;
    out[14] = a0 * b14 + a2 * b10 + a3 * b9 + a4 * b8 + a8 * b4 + a9 * b3 + a10 * b2 + a14 * b0;
    out[15] =
        a0 * b15 +
        a1 * b14 +
        a2 * b13 +
        a3 * b12 +
        a4 * b11 +
        a5 * b10 +
        a6 * b9 +
        a7 * b8 +
        a8 * b7 +
        a9 * b6 +
        a10 * b5 -
        a11 * b4 -
        a12 * b3 -
        a13 * b2 -
        a14 * b1 +
        a15 * b0;
    return out;
}

/**
 * Alias for {@link pga3d.wedge}
 * @function
 */
export const meet = wedge;

/**
 * Regressive product of two multivectors, the join. Two points join into the line from a to b,
 * a line and a point join into a plane. Uses the Hodge dual convention of Dorst and De Keninck.
 *
 * @param out the receiving multivector
 * @param a the first operand
 * @param b the second operand
 * @returns out
 */
export function vee(out: PGA3D, a: Const<PGA3D>, b: Const<PGA3D>): PGA3D {
    const a0 = a[0];
    const a1 = a[1];
    const a2 = a[2];
    const a3 = a[3];
    const a4 = a[4];
    const a5 = a[5];
    const a6 = a[6];
    const a7 = a[7];
    const a8 = a[8];
    const a9 = a[9];
    const a10 = a[10];
    const a11 = a[11];
    const a12 = a[12];
    const a13 = a[13];
    const a14 = a[14];
    const a15 = a[15];
    const b0 = b[0];
    const b1 = b[1];
    const b2 = b[2];
    const b3 = b[3];
    const b4 = b[4];
    const b5 = b[5];
    const b6 = b[6];
    const b7 = b[7];
    const b8 = b[8];
    const b9 = b[9];
    const b10 = b[10];
    const b11 = b[11];
    const b12 = b[12];
    const b13 = b[13];
    const b14 = b[14];
    const b15 = b[15];
    out[0] =
        a0 * b15 +
        a1 * b14 +
        a2 * b13 +
        a3 * b12 +
        a4 * b11 +
        a5 * b10 +
        a6 * b9 +
        a7 * b8 +
        a8 * b7 +
        a9 * b6 +
        a10 * b5 -
        a11 * b4 -
        a12 * b3 -
        a13 * b2 -
        a14 * b1 +
        a15 * b0;
    out[1] = a1 * b15 - a5 * b13 - a6 * b12 - a7 * b11 - a11 * b7 - a12 * b6 - a13 * b5 + a15 * b1;
    out[2] = a2 * b15 + a5 * b14 - a8 * b12 + a9 * b11 + a11 * b9 - a12 * b8 + a14 * b5 + a15 * b2;
    out[3] = a3 * b15 + a6 * b14 + a8 * b13 - a10 * b11 - a11 * b10 + a13 * b8 + a14 * b6 + a15 * b3;
    out[4] = a4 * b15 + a7 * b14 - a9 * b13 + a10 * b12 + a12 * b10 - a13 * b9 + a14 * b7 + a15 * b4;
    out[5] = a5 * b15 - a11 * b12 + a12 * b11 + a15 * b5;
    out[6] = a6 * b15 + a11 * b13 - a13 * b11 + a15 * b6;
    out[7] = a7 * b15 - a12 * b13 + a13 * b12 + a15 * b7;
    out[8] = a8 * b15 - a11 * b14 + a14 * b11 + a15 * b8;
    out[9] = a9 * b15 - a12 * b14 + a14 * b12 + a15 * b9;
    out[10] = a10 * b15 - a13 * b14 + a14 * b13 + a15 * b10;
    out[11] = a11 * b15 + a15 * b11;
    out[12] = a12 * b15 + a15 * b12;
    out[13] = a13 * b15 + a15 * b13;
    out[14] = a14 * b15 + a15 * b14;
    out[15] = a15 * b15;
    return out;
}

/**
 * Alias for {@link pga3d.vee}
 * @function
 */
export const join = vee;

/**
 * Symmetric inner product of two multivectors, keeping the terms of grade |grade(a) - grade(b)|.
 * For two normalized planes the scalar part is the cosine of their angle.
 *
 * @param out the receiving multivector
 * @param a the first operand
 * @param b the second operand
 * @returns out
 */
export function dot(out: PGA3D, a: Const<PGA3D>, b: Const<PGA3D>): PGA3D {
    const a0 = a[0];
    const a1 = a[1];
    const a2 = a[2];
    const a3 = a[3];
    const a4 = a[4];
    const a5 = a[5];
    const a6 = a[6];
    const a7 = a[7];
    const a8 = a[8];
    const a9 = a[9];
    const a10 = a[10];
    const a11 = a[11];
    const a12 = a[12];
    const a13 = a[13];
    const a14 = a[14];
    const a15 = a[15];
    const b0 = b[0];
    const b1 = b[1];
    const b2 = b[2];
    const b3 = b[3];
    const b4 = b[4];
    const b5 = b[5];
    const b6 = b[6];
    const b7 = b[7];
    const b8 = b[8];
    const b9 = b[9];
    const b10 = b[10];
    const b11 = b[11];
    const b12 = b[12];
    const b13 = b[13];
    const b14 = b[14];
    const b15 = b[15];
    out[0] = a0 * b0 + a2 * b2 + a3 * b3 + a4 * b4 - a8 * b8 - a9 * b9 - a10 * b10 - a14 * b14;
    out[1] =
        a0 * b1 +
        a1 * b0 -
        a2 * b5 -
        a3 * b6 -
        a4 * b7 +
        a5 * b2 +
        a6 * b3 +
        a7 * b4 +
        a8 * b11 +
        a9 * b12 +
        a10 * b13 +
        a11 * b8 +
        a12 * b9 +
        a13 * b10 +
        a14 * b15 -
        a15 * b14;
    out[2] = a0 * b2 + a2 * b0 - a3 * b8 + a4 * b9 + a8 * b3 - a9 * b4 - a10 * b14 - a14 * b10;
    out[3] = a0 * b3 + a2 * b8 + a3 * b0 - a4 * b10 - a8 * b2 - a9 * b14 + a10 * b4 - a14 * b9;
    out[4] = a0 * b4 - a2 * b9 + a3 * b10 + a4 * b0 - a8 * b14 + a9 * b2 - a10 * b3 - a14 * b8;
    out[5] = a0 * b5 - a3 * b11 + a4 * b12 + a5 * b0 - a10 * b15 - a11 * b3 + a12 * b4 - a15 * b10;
    out[6] = a0 * b6 + a2 * b11 - a4 * b13 + a6 * b0 - a9 * b15 + a11 * b2 - a13 * b4 - a15 * b9;
    out[7] = a0 * b7 - a2 * b12 + a3 * b13 + a7 * b0 - a8 * b15 - a12 * b2 + a13 * b3 - a15 * b8;
    out[8] = a0 * b8 + a4 * b14 + a8 * b0 + a14 * b4;
    out[9] = a0 * b9 + a3 * b14 + a9 * b0 + a14 * b3;
    out[10] = a0 * b10 + a2 * b14 + a10 * b0 + a14 * b2;
    out[11] = a0 * b11 + a4 * b15 + a11 * b0 - a15 * b4;
    out[12] = a0 * b12 + a3 * b15 + a12 * b0 - a15 * b3;
    out[13] = a0 * b13 + a2 * b15 + a13 * b0 - a15 * b2;
    out[14] = a0 * b14 + a14 * b0;
    out[15] = a0 * b15 + a15 * b0;
    return out;
}

/**
 * Reverse of a multivector, flipping the sign of grades 2 and 3. The inverse of a normalized motor.
 *
 * @param out the receiving multivector
 * @param a the multivector to reverse
 * @returns out
 */
export function reverse(out: PGA3D, a: Const<PGA3D>): PGA3D {
    out[0] = a[0];
    out[1] = a[1];
    out[2] = a[2];
    out[3] = a[3];
    out[4] = a[4];
    out[5] = -a[5];
    out[6] = -a[6];
    out[7] = -a[7];
    out[8] = -a[8];
    out[9] = -a[9];
    out[10] = -a[10];
    out[11] = -a[11];
    out[12] = -a[12];
    out[13] = -a[13];
    out[14] = -a[14];
    out[15] = a[15];
    return out;
}

/**
 * Grade involution of a multivector, flipping the sign of the odd grades
 *
 * @param out the receiving multivector
 * @param a the source multivector
 * @returns out
 */
export function involute(out: PGA3D, a: Const<PGA3D>): PGA3D {
    out[0] = a[0];
    out[1] = -a[1];
    out[2] = -a[2];
    out[3] = -a[3];
    out[4] = -a[4];
    out[5] = a[5];
    out[6] = a[6];
    out[7] = a[7];
    out[8] = a[8];
    out[9] = a[9];
    out[10] = a[10];
    out[11] = -a[11];
    out[12] = -a[12];
    out[13] = -a[13];
    out[14] = -a[14];
    out[15] = a[15];
    return out;
}

/**
 * Clifford conjugate of a multivector, flipping the sign of grades 1 and 2
 *
 * @param out the receiving multivector
 * @param a the source multivector
 * @returns out
 */
export function conjugate(out: PGA3D, a: Const<PGA3D>): PGA3D {
    out[0] = a[0];
    out[1] = -a[1];
    out[2] = -a[2];
    out[3] = -a[3];
    out[4] = -a[4];
    out[5] = -a[5];
    out[6] = -a[6];
    out[7] = -a[7];
    out[8] = -a[8];
    out[9] = -a[9];
    out[10] = -a[10];
    out[11] = a[11];
    out[12] = a[12];
    out[13] = a[13];
    out[14] = a[14];
    out[15] = a[15];
    return out;
}

/**
 * Hodge dual of a multivector, mapping each blade b to the blade d with b ^ d = e0123.
 * Maps a point to its dual plane and a line to its dual line.
 *
 * @param out the receiving multivector
 * @param a the source multivector
 * @returns out
 */
export function dual(out: PGA3D, a: Const<PGA3D>): PGA3D {
    const a0 = a[0];
    const a1 = a[1];
    const a2 = a[2];
    const a3 = a[3];
    const a4 = a[4];
    const a5 = a[5];
    const a6 = a[6];
    const a7 = a[7];
    const a8 = a[8];
    const a9 = a[9];
    const a10 = a[10];
    const a11 = a[11];
    const a12 = a[12];
    const a13 = a[13];
    const a14 = a[14];
    const a15 = a[15];
    out[0] = a15;
    out[1] = -a14;
    out[2] = -a13;
    out[3] = -a12;
    out[4] = -a11;
    out[5] = a10;
    out[6] = a9;
    out[7] = a8;
    out[8] = a7;
    out[9] = a6;
    out[10] = a5;
    out[11] = a4;
    out[12] = a3;
    out[13] = a2;
    out[14] = a1;
    out[15] = a0;
    return out;
}

/**
 * Inverse of the Hodge dual, so that undual(dual(a)) equals a
 *
 * @param out the receiving multivector
 * @param a the source multivector
 * @returns out
 */
export function undual(out: PGA3D, a: Const<PGA3D>): PGA3D {
    const a0 = a[0];
    const a1 = a[1];
    const a2 = a[2];
    const a3 = a[3];
    const a4 = a[4];
    const a5 = a[5];
    const a6 = a[6];
    const a7 = a[7];
    const a8 = a[8];
    const a9 = a[9];
    const a10 = a[10];
    const a11 = a[11];
    const a12 = a[12];
    const a13 = a[13];
    const a14 = a[14];
    const a15 = a[15];
    out[0] = a15;
    out[1] = a14;
    out[2] = a13;
    out[3] = a12;
    out[4] = a11;
    out[5] = a10;
    out[6] = a9;
    out[7] = a8;
    out[8] = a7;
    out[9] = a6;
    out[10] = a5;
    out[11] = -a4;
    out[12] = -a3;
    out[13] = -a2;
    out[14] = -a1;
    out[15] = a0;
    return out;
}

/**
 * Calculates the norm of a multivector, the square root of the scalar part of a * reverse(a).
 * This is the weight of a point, the length of a plane normal and the length of a line direction.
 * Ideal elements have norm 0, see {@link pga3d.idealNorm}.
 *
 * @param a the multivector
 * @returns norm of a
 */
export function norm(a: Const<PGA3D>): number {
    return Math.sqrt(
        Math.abs(
            a[0] * a[0] + a[2] * a[2] + a[3] * a[3] + a[4] * a[4] + a[8] * a[8] + a[9] * a[9] + a[10] * a[10] + a[14] * a[14],
        ),
    );
}

/**
 * Calculates the ideal norm of a multivector, the norm of its dual.
 * This is the length of a direction, the distance term of a plane and the moment of a line.
 *
 * @param a the multivector
 * @returns ideal norm of a
 */
export function idealNorm(a: Const<PGA3D>): number {
    return Math.sqrt(
        Math.abs(
            a[1] * a[1] + a[5] * a[5] + a[6] * a[6] + a[7] * a[7] + a[11] * a[11] + a[12] * a[12] + a[13] * a[13] + a[15] * a[15],
        ),
    );
}

/**
 * Normalizes a versor so that a * reverse(a) = 1. Works for planes, points, lines and motors,
 * removing the e0123 drift of a motor like quat2.normalize does for a dual quaternion.
 *
 * @param out the receiving multivector
 * @param a the versor to normalize
 * @returns out
 */
export function normalize(out: PGA3D, a: Const<PGA3D>): PGA3D {
    const a0 = a[0];
    const a1 = a[1];
    const a2 = a[2];
    const a3 = a[3];
    const a4 = a[4];
    const a5 = a[5];
    const a6 = a[6];
    const a7 = a[7];
    const a8 = a[8];
    const a9 = a[9];
    const a10 = a[10];
    const a11 = a[11];
    const a12 = a[12];
    const a13 = a[13];
    const a14 = a[14];
    const a15 = a[15];
    // a * reverse(a) = s + t e0123 for any versor
    const s = a0 * a0 + a2 * a2 + a3 * a3 + a4 * a4 + a8 * a8 + a9 * a9 + a10 * a10 + a14 * a14;
    if (s === 0) {
        return copy(out, a);
    }
    const t = 2 * (a0 * a15 - a5 * a10 - a6 * a9 - a7 * a8 - a1 * a14 - a2 * a13 - a3 * a12 - a4 * a11);
    // multiply by (s + t e0123) ^ -1/2 = alpha + beta e0123, where e0123 anticommutes with the odd grades
    const alpha = 1 / Math.sqrt(s);
    const beta = (-t * alpha) / (2 * s);
    out[0] = alpha * a0;
    out[1] = alpha * a1 - beta * a14;
    out[2] = alpha * a2;
    out[3] = alpha * a3;
    out[4] = alpha * a4;
    out[5] = alpha * a5 - beta * a10;
    out[6] = alpha * a6 - beta * a9;
    out[7] = alpha * a7 - beta * a8;
    out[8] = alpha * a8;
    out[9] = alpha * a9;
    out[10] = alpha * a10;
    out[11] = alpha * a11 - beta * a4;
    out[12] = alpha * a12 - beta * a3;
    out[13] = alpha * a13 - beta * a2;
    out[14] = alpha * a14;
    out[15] = alpha * a15 + beta * a0;
    return out;
}

/**
 * Calculates the inverse of a versor, so that multiply(out, a) = 1.
 * Works for planes, points, lines and motors. If the versor is normalized, reverse is cheaper.
 *
 * @param out the receiving multivector
 * @param a the versor to invert
 * @returns out
 */
export function invert(out: PGA3D, a: Const<PGA3D>): PGA3D {
    const a0 = a[0];
    const a1 = a[1];
    const a2 = a[2];
    const a3 = a[3];
    const a4 = a[4];
    const a5 = a[5];
    const a6 = a[6];
    const a7 = a[7];
    const a8 = a[8];
    const a9 = a[9];
    const a10 = a[10];
    const a11 = a[11];
    const a12 = a[12];
    const a13 = a[13];
    const a14 = a[14];
    const a15 = a[15];
    // inverse = reverse(a) * (s + t e0123) ^ -1 = reverse(a) * (1/s - t/s^2 e0123), multiplied on the right,
    // so the e0123 term flips sign on the odd grades, which anticommute with e0123
    const s = a0 * a0 + a2 * a2 + a3 * a3 + a4 * a4 + a8 * a8 + a9 * a9 + a10 * a10 + a14 * a14;
    const t = 2 * (a0 * a15 - a5 * a10 - a6 * a9 - a7 * a8 - a1 * a14 - a2 * a13 - a3 * a12 - a4 * a11);
    const alpha = 1 / s;
    const beta = -t / (s * s);
    out[0] = alpha * a0;
    out[1] = alpha * a1 - beta * a14;
    out[2] = alpha * a2;
    out[3] = alpha * a3;
    out[4] = alpha * a4;
    out[5] = -alpha * a5 + beta * a10;
    out[6] = -alpha * a6 + beta * a9;
    out[7] = -alpha * a7 + beta * a8;
    out[8] = -alpha * a8;
    out[9] = -alpha * a9;
    out[10] = -alpha * a10;
    out[11] = -alpha * a11 + beta * a4;
    out[12] = -alpha * a12 + beta * a3;
    out[13] = -alpha * a13 + beta * a2;
    out[14] = -alpha * a14;
    out[15] = alpha * a15 + beta * a0;
    return out;
}

/**
 * Sandwich product m * x * reverse(m). Applies the motor m to a point, line, plane or other motor x.
 *
 * @param out the receiving multivector
 * @param m the motor to apply
 * @param x the multivector to transform
 * @returns out
 */
export function sandwich(out: PGA3D, m: Const<PGA3D>, x: Const<PGA3D>): PGA3D {
    const m0 = m[0];
    const m1 = m[1];
    const m2 = m[2];
    const m3 = m[3];
    const m4 = m[4];
    const m5 = m[5];
    const m6 = m[6];
    const m7 = m[7];
    const m8 = m[8];
    const m9 = m[9];
    const m10 = m[10];
    const m11 = m[11];
    const m12 = m[12];
    const m13 = m[13];
    const m14 = m[14];
    const m15 = m[15];
    const x0 = x[0];
    const x1 = x[1];
    const x2 = x[2];
    const x3 = x[3];
    const x4 = x[4];
    const x5 = x[5];
    const x6 = x[6];
    const x7 = x[7];
    const x8 = x[8];
    const x9 = x[9];
    const x10 = x[10];
    const x11 = x[11];
    const x12 = x[12];
    const x13 = x[13];
    const x14 = x[14];
    const x15 = x[15];
    const t0 = m0 * x0 + m2 * x2 + m3 * x3 + m4 * x4 - m8 * x8 - m9 * x9 - m10 * x10 - m14 * x14;
    const t1 =
        m0 * x1 +
        m1 * x0 -
        m2 * x5 -
        m3 * x6 -
        m4 * x7 +
        m5 * x2 +
        m6 * x3 +
        m7 * x4 +
        m8 * x11 +
        m9 * x12 +
        m10 * x13 +
        m11 * x8 +
        m12 * x9 +
        m13 * x10 +
        m14 * x15 -
        m15 * x14;
    const t2 = m0 * x2 + m2 * x0 - m3 * x8 + m4 * x9 + m8 * x3 - m9 * x4 - m10 * x14 - m14 * x10;
    const t3 = m0 * x3 + m2 * x8 + m3 * x0 - m4 * x10 - m8 * x2 - m9 * x14 + m10 * x4 - m14 * x9;
    const t4 = m0 * x4 - m2 * x9 + m3 * x10 + m4 * x0 - m8 * x14 + m9 * x2 - m10 * x3 - m14 * x8;
    const t5 =
        m0 * x5 +
        m1 * x2 -
        m2 * x1 -
        m3 * x11 +
        m4 * x12 +
        m5 * x0 -
        m6 * x8 +
        m7 * x9 +
        m8 * x6 -
        m9 * x7 -
        m10 * x15 -
        m11 * x3 +
        m12 * x4 +
        m13 * x14 -
        m14 * x13 -
        m15 * x10;
    const t6 =
        m0 * x6 +
        m1 * x3 +
        m2 * x11 -
        m3 * x1 -
        m4 * x13 +
        m5 * x8 +
        m6 * x0 -
        m7 * x10 -
        m8 * x5 -
        m9 * x15 +
        m10 * x7 +
        m11 * x2 +
        m12 * x14 -
        m13 * x4 -
        m14 * x12 -
        m15 * x9;
    const t7 =
        m0 * x7 +
        m1 * x4 -
        m2 * x12 +
        m3 * x13 -
        m4 * x1 -
        m5 * x9 +
        m6 * x10 +
        m7 * x0 -
        m8 * x15 +
        m9 * x5 -
        m10 * x6 +
        m11 * x14 -
        m12 * x2 +
        m13 * x3 -
        m14 * x11 -
        m15 * x8;
    const t8 = m0 * x8 + m2 * x3 - m3 * x2 + m4 * x14 + m8 * x0 + m9 * x10 - m10 * x9 + m14 * x4;
    const t9 = m0 * x9 - m2 * x4 + m3 * x14 + m4 * x2 - m8 * x10 + m9 * x0 + m10 * x8 + m14 * x3;
    const t10 = m0 * x10 + m2 * x14 + m3 * x4 - m4 * x3 + m8 * x9 - m9 * x8 + m10 * x0 + m14 * x2;
    const t11 =
        m0 * x11 -
        m1 * x8 +
        m2 * x6 -
        m3 * x5 +
        m4 * x15 -
        m5 * x3 +
        m6 * x2 -
        m7 * x14 -
        m8 * x1 +
        m9 * x13 -
        m10 * x12 +
        m11 * x0 +
        m12 * x10 -
        m13 * x9 +
        m14 * x7 -
        m15 * x4;
    const t12 =
        m0 * x12 -
        m1 * x9 -
        m2 * x7 +
        m3 * x15 +
        m4 * x5 +
        m5 * x4 -
        m6 * x14 -
        m7 * x2 -
        m8 * x13 -
        m9 * x1 +
        m10 * x11 -
        m11 * x10 +
        m12 * x0 +
        m13 * x8 +
        m14 * x6 -
        m15 * x3;
    const t13 =
        m0 * x13 -
        m1 * x10 +
        m2 * x15 +
        m3 * x7 -
        m4 * x6 -
        m5 * x14 -
        m6 * x4 +
        m7 * x3 +
        m8 * x12 -
        m9 * x11 -
        m10 * x1 +
        m11 * x9 -
        m12 * x8 +
        m13 * x0 +
        m14 * x5 -
        m15 * x2;
    const t14 = m0 * x14 + m2 * x10 + m3 * x9 + m4 * x8 + m8 * x4 + m9 * x3 + m10 * x2 + m14 * x0;
    const t15 =
        m0 * x15 +
        m1 * x14 +
        m2 * x13 +
        m3 * x12 +
        m4 * x11 +
        m5 * x10 +
        m6 * x9 +
        m7 * x8 +
        m8 * x7 +
        m9 * x6 +
        m10 * x5 -
        m11 * x4 -
        m12 * x3 -
        m13 * x2 -
        m14 * x1 +
        m15 * x0;
    out[0] = t0 * m0 + t2 * m2 + t3 * m3 + t4 * m4 + t8 * m8 + t9 * m9 + t10 * m10 + t14 * m14;
    out[1] =
        t0 * m1 +
        t1 * m0 +
        t2 * m5 +
        t3 * m6 +
        t4 * m7 +
        t5 * m2 +
        t6 * m3 +
        t7 * m4 -
        t8 * m11 -
        t9 * m12 -
        t10 * m13 -
        t11 * m8 -
        t12 * m9 -
        t13 * m10 +
        t14 * m15 +
        t15 * m14;
    out[2] = t0 * m2 + t2 * m0 + t3 * m8 - t4 * m9 + t8 * m3 - t9 * m4 + t10 * m14 + t14 * m10;
    out[3] = t0 * m3 - t2 * m8 + t3 * m0 + t4 * m10 - t8 * m2 + t9 * m14 + t10 * m4 + t14 * m9;
    out[4] = t0 * m4 + t2 * m9 - t3 * m10 + t4 * m0 + t8 * m14 + t9 * m2 - t10 * m3 + t14 * m8;
    out[5] =
        -t0 * m5 +
        t1 * m2 -
        t2 * m1 +
        t3 * m11 -
        t4 * m12 +
        t5 * m0 +
        t6 * m8 -
        t7 * m9 -
        t8 * m6 +
        t9 * m7 -
        t10 * m15 -
        t11 * m3 +
        t12 * m4 -
        t13 * m14 +
        t14 * m13 +
        t15 * m10;
    out[6] =
        -t0 * m6 +
        t1 * m3 -
        t2 * m11 -
        t3 * m1 +
        t4 * m13 -
        t5 * m8 +
        t6 * m0 +
        t7 * m10 +
        t8 * m5 -
        t9 * m15 -
        t10 * m7 +
        t11 * m2 -
        t12 * m14 -
        t13 * m4 +
        t14 * m12 +
        t15 * m9;
    out[7] =
        -t0 * m7 +
        t1 * m4 +
        t2 * m12 -
        t3 * m13 -
        t4 * m1 +
        t5 * m9 -
        t6 * m10 +
        t7 * m0 -
        t8 * m15 -
        t9 * m5 +
        t10 * m6 -
        t11 * m14 -
        t12 * m2 +
        t13 * m3 +
        t14 * m11 +
        t15 * m8;
    out[8] = -t0 * m8 + t2 * m3 - t3 * m2 - t4 * m14 + t8 * m0 - t9 * m10 + t10 * m9 + t14 * m4;
    out[9] = -t0 * m9 - t2 * m4 - t3 * m14 + t4 * m2 + t8 * m10 + t9 * m0 - t10 * m8 + t14 * m3;
    out[10] = -t0 * m10 - t2 * m14 + t3 * m4 - t4 * m3 - t8 * m9 + t9 * m8 + t10 * m0 + t14 * m2;
    out[11] =
        -t0 * m11 +
        t1 * m8 -
        t2 * m6 +
        t3 * m5 +
        t4 * m15 -
        t5 * m3 +
        t6 * m2 +
        t7 * m14 -
        t8 * m1 -
        t9 * m13 +
        t10 * m12 +
        t11 * m0 -
        t12 * m10 +
        t13 * m9 -
        t14 * m7 -
        t15 * m4;
    out[12] =
        -t0 * m12 +
        t1 * m9 +
        t2 * m7 +
        t3 * m15 -
        t4 * m5 +
        t5 * m4 +
        t6 * m14 -
        t7 * m2 +
        t8 * m13 -
        t9 * m1 -
        t10 * m11 +
        t11 * m10 +
        t12 * m0 -
        t13 * m8 -
        t14 * m6 -
        t15 * m3;
    out[13] =
        -t0 * m13 +
        t1 * m10 +
        t2 * m15 -
        t3 * m7 +
        t4 * m6 +
        t5 * m14 -
        t6 * m4 +
        t7 * m3 -
        t8 * m12 +
        t9 * m11 -
        t10 * m1 -
        t11 * m9 +
        t12 * m8 +
        t13 * m0 -
        t14 * m5 -
        t15 * m2;
    out[14] = -t0 * m14 - t2 * m10 - t3 * m9 - t4 * m8 + t8 * m4 + t9 * m3 + t10 * m2 + t14 * m0;
    out[15] =
        t0 * m15 -
        t1 * m14 -
        t2 * m13 -
        t3 * m12 -
        t4 * m11 -
        t5 * m10 -
        t6 * m9 -
        t7 * m8 -
        t8 * m7 -
        t9 * m6 -
        t10 * m5 -
        t11 * m4 -
        t12 * m3 -
        t13 * m2 -
        t14 * m1 +
        t15 * m0;
    return out;
}

/**
 * Exponential of a bivector, giving the motor that rotates and translates along the line it describes.
 * exp of -rad/2 times a normalized line rotates by rad around that line, exp of -0.5 times an ideal line translates.
 *
 * @param out the receiving multivector
 * @param b a bivector
 * @returns out
 */
export function exp(out: PGA3D, b: Const<PGA3D>): PGA3D {
    const b01 = b[5];
    const b02 = b[6];
    const b03 = b[7];
    const b12 = b[8];
    const b31 = b[9];
    const b23 = b[10];
    // l is the squared rotation angle and m the pitch term, b * b = -l + 2 m e0123
    const l = b12 * b12 + b31 * b31 + b23 * b23;
    const m = b01 * b23 + b02 * b31 + b03 * b12;
    const a = Math.sqrt(l);
    const c = Math.cos(a);
    // s = sin(a) / a and t = m (cos(a) - s) / l, with Taylor series below l = 1e-3 where they are accurate to 1e-16
    let s: number;
    let t: number;
    if (l < 1e-3) {
        s = 1 - (l / 6) * (1 - (l / 20) * (1 - l / 42));
        t = m * (-1 / 3 + l * (1 / 30 - l * (1 / 840 - l / 45360)));
    } else {
        s = Math.sin(a) / a;
        t = (m * (c - s)) / l;
    }
    identity(out);
    out[0] = c;
    out[5] = s * b01 + t * b23;
    out[6] = s * b02 + t * b31;
    out[7] = s * b03 + t * b12;
    out[8] = s * b12;
    out[9] = s * b31;
    out[10] = s * b23;
    out[15] = m * s;
    return out;
}

/**
 * Logarithm of a normalized motor, the inverse of {@link pga3d.exp}. Returns the bivector b with exp(b) = m.
 * The result rotates by at most 2 pi. Near a rotation by 2 pi, a scalar part close to -1, the log is singular.
 *
 * @param out the receiving multivector
 * @param m a normalized motor
 * @returns out
 */
export function log(out: PGA3D, m: Const<PGA3D>): PGA3D {
    const s = m[0];
    const m01 = m[5];
    const m02 = m[6];
    const m03 = m[7];
    const m12 = m[8];
    const m31 = m[9];
    const m23 = m[10];
    const m0123 = m[15];
    // m = cos(a) + sin(a) u + ideal terms, with a the half rotation angle and u the unit rotation axis
    const sigma2 = m12 * m12 + m31 * m31 + m23 * m23;
    const sigma = Math.sqrt(sigma2);
    const a = Math.atan2(sigma, s);
    // b = a / sin(a) scales the bivector and c = m0123 (1 - a cot(a)) / sin(a)^2 corrects the moment for the pitch
    let b: number;
    let c: number;
    if (sigma === 0) {
        // no rotation, a pure translation. A scalar part of -1 is a full turn, the same as the identity
        b = s < 0 ? -1 : 1;
        c = 0;
    } else if (a * a < 1e-4) {
        // Taylor series of c in a^2, accurate to 1e-16 below the cutoff
        const a2 = a * a;
        b = a / sigma;
        c = m0123 * (1 / 3 + a2 * (2 / 15 + a2 * (2 / 63 + (a2 * 4) / 675)));
    } else {
        b = a / sigma;
        c = (m0123 * (1 - (a * s) / sigma)) / sigma2;
    }
    for (let i = 0; i < 16; i++) {
        out[i] = 0;
    }
    out[5] = c * m23 + b * m01;
    out[6] = c * m31 + b * m02;
    out[7] = c * m12 + b * m03;
    out[8] = b * m12;
    out[9] = b * m31;
    out[10] = b * m23;
    return out;
}

/**
 * Returns a string representation of a multivector, listing the non zero blades
 *
 * @param a multivector to represent as a string
 * @returns string representation of the multivector
 */
export function str(a: Const<PGA3D>): string {
    const blades = [
        '1',
        'e0',
        'e1',
        'e2',
        'e3',
        'e01',
        'e02',
        'e03',
        'e12',
        'e31',
        'e23',
        'e021',
        'e013',
        'e032',
        'e123',
        'e0123',
    ];
    const terms: string[] = [];
    for (let i = 0; i < 16; i++) {
        if (a[i] !== 0) {
            terms.push(i === 0 ? `${a[i]}` : `${a[i]}${blades[i]}`);
        }
    }
    return `pga3d(${terms.length === 0 ? '0' : terms.join(' + ')})`;
}

/**
 * Returns whether or not the multivectors have exactly the same elements in the same position (when compared with ===)
 *
 * @param a the first multivector
 * @param b the second multivector
 * @returns true if the multivectors are equal, false otherwise
 */
export function exactEquals(a: Const<PGA3D>, b: Const<PGA3D>): boolean {
    for (let i = 0; i < 16; i++) {
        if (a[i] !== b[i]) {
            return false;
        }
    }
    return true;
}

/**
 * Returns whether or not the multivectors have approximately the same elements in the same position
 *
 * @param a the first multivector
 * @param b the second multivector
 * @returns true if the multivectors are equal, false otherwise
 */
export function equals(a: Const<PGA3D>, b: Const<PGA3D>): boolean {
    for (let i = 0; i < 16; i++) {
        if (Math.abs(a[i] - b[i]) > EPSILON * Math.max(1.0, Math.abs(a[i]), Math.abs(b[i]))) {
            return false;
        }
    }
    return true;
}
