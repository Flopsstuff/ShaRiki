//! Tiny 3-vector helpers (f32, no dependencies).

pub(crate) type V3 = [f32; 3];

#[inline]
pub(crate) fn add(a: V3, b: V3) -> V3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

#[inline]
pub(crate) fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

#[inline]
pub(crate) fn scale(a: V3, k: f32) -> V3 {
    [a[0] * k, a[1] * k, a[2] * k]
}

#[inline]
pub(crate) fn dot(a: V3, b: V3) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

#[inline]
pub(crate) fn cross(a: V3, b: V3) -> V3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

#[inline]
pub(crate) fn norm(a: V3) -> f32 {
    dot(a, a).sqrt()
}

/// Unit vector along `a`, or `None` if `a` is (nearly) zero or not finite.
#[inline]
pub(crate) fn normalize(a: V3) -> Option<V3> {
    let n = norm(a);
    if n.is_finite() && n > 1e-6 {
        Some(scale(a, 1.0 / n))
    } else {
        None
    }
}

/// Rodrigues rotation of `v` by `angle` radians about the unit vector `k`.
#[inline]
pub(crate) fn rotate(v: V3, k: V3, angle: f32) -> V3 {
    let (s, c) = angle.sin_cos();
    let kxv = cross(k, v);
    let kdv = dot(k, v);
    [
        v[0] * c + kxv[0] * s + k[0] * kdv * (1.0 - c),
        v[1] * c + kxv[1] * s + k[1] * kdv * (1.0 - c),
        v[2] * c + kxv[2] * s + k[2] * kdv * (1.0 - c),
    ]
}

/// How a world-fixed vector `v` (expressed in the sensor frame) changes when the sensor
/// turns at `omega_dps` (sensor frame, deg/s) for `dt` seconds: it rotates by the opposite
/// angle, `-|ω|·dt` about `ω̂`.
#[inline]
pub(crate) fn rotate_world_vec(v: V3, omega_dps: V3, dt: f32) -> V3 {
    let rate = norm(omega_dps);
    if !(rate.is_finite() && rate > 1e-9) {
        return v;
    }
    let k = scale(omega_dps, 1.0 / rate);
    rotate(v, k, -(rate * dt).to_radians())
}
