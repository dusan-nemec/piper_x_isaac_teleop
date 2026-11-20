
import math

# --- quaternion helpers, (x, y, z, w) -------------------------------------------
def q_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def q_from_rotvec(v):
    angle = math.sqrt(sum(c * c for c in v))
    if angle < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    s = math.sin(angle / 2) / angle
    return (v[0] * s, v[1] * s, v[2] * s, math.cos(angle / 2))


def q_from_rpy(r, p, y):
    return q_mul(q_from_rotvec((0, 0, y)), q_mul(q_from_rotvec((0, p, 0)),
                                                 q_from_rotvec((r, 0, 0))))


def q_to_rotvec(q):
    x, y, z, w = q
    if w < 0:  # shortest path
        x, y, z, w = -x, -y, -z, -w
    n = math.sqrt(x * x + y * y + z * z)
    if n < 1e-12:
        return (0.0, 0.0, 0.0)
    angle = 2 * math.atan2(n, w)
    return (x / n * angle, y / n * angle, z / n * angle)


def q_conj(q):
    return (-q[0], -q[1], -q[2], q[3])


def q_normalize(q):
    n = math.sqrt(sum(c * c for c in q))
    return tuple(c / n for c in q)