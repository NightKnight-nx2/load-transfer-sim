"""
Bicycle model (ön+arka tek-iz eşdeğeri) — yanal dinamik.

Durumlar:  [X, Y, psi, vy, r, ay_f]
    vy, r : yanal hız ve yaw hızı (asıl dinamik DOF'lar)
    ay_f  : yük transferinde kullanılan, birinci derece gecikmeli yanal ivme
            (roll geçici davranışı; ayrıca ay -> Fz -> Fy -> ay cebirsel
            döngüsünü kırar). Kararlı halde ay_f = ay.
Boylamsal hız vx(t) ÖNCEDEN VERİLİR (hız profili) — "aynı hız profili"
gereksinimi. Bundan gereken boylamsal kuvvet (m*ax + sürükleme) fren dağılımı
(ön) / RWD (arka) ile akslara verilir ve friction-ellipse yoluyla yanal
kuvveti azaltır (combined grip).

Aks başına iki tekerlek vardır: aynı kayma açısı, farklı Fz -> her tekerin
Fy'si ayrı Pacejka ile hesaplanır, aks Fy'si toplamdır (yük hassasiyeti
burada devreye girer). Kayma açısı Sistem 7 ile aynı tanımdadır:
    alpha_f = atan2(vy + a r, vx) - delta,   alpha_r = atan2(vy - b r, vx)
ve Fy_tekerlek = -D sin(...) (hıza ters).
"""
from __future__ import annotations

from dataclasses import dataclass, field, replace

import numpy as np

from load_transfer_sim.load_transfer import RollStiffness, wheel_loads
from load_transfer_sim.tire_model import (
    PacejkaParams, lateral_force_with_fx, peak_lateral_force,
    peak_longitudinal_force, peak_slip_angle,
)


@dataclass
class BicycleParams:
    mass: float = 1400.0
    Iz: float = 2100.0
    a: float = 1.35
    b: float = 1.35
    track_f: float = 1.55
    track_r: float = 1.55
    h_cg: float = 0.50
    roll: RollStiffness = field(default_factory=RollStiffness)
    roll_tau: float = 0.06          # s, yanal ivme/roll gecikmesi
    brake_bias_front: float = 0.65  # fren kuvvetinin ön aks payı
    drag_coeff: float = 0.32
    frontal_area: float = 2.2
    air_density: float = 1.225
    rolling_resistance: float = 0.014
    tire_f: PacejkaParams = field(default_factory=PacejkaParams)
    tire_r: PacejkaParams = field(default_factory=PacejkaParams)
    g: float = 9.81

    @property
    def L(self):
        return self.a + self.b

    @classmethod
    def from_sistem7(cls, **overrides) -> "BicycleParams":
        """Kütle, Iz, iz, h_cg, sürükleme, lastikler VehicleParams'tan alınır."""
        from load_transfer_sim.vehicle_model import VehicleParams
        v = VehicleParams()
        base = dict(mass=v.mass, Iz=v.Iz, a=v.a, b=v.b, track_f=v.track_width,
                    track_r=v.track_width, h_cg=v.cg_height,
                    brake_bias_front=v.brake_bias_front, drag_coeff=v.drag_coeff,
                    frontal_area=v.frontal_area, air_density=v.air_density,
                    rolling_resistance=v.rolling_resistance,
                    tire_f=v.tire_front, tire_r=v.tire_rear)
        base.update(overrides)
        return cls(**base)

    def with_roll_share(self, front_share: float) -> "BicycleParams":
        return replace(self, roll=RollStiffness.from_share(front_share, self.roll.K_f + self.roll.K_r))


def slip_angles(p, vx, vy, r, delta):
    vx = max(vx, 1.0)
    return np.arctan2(vy + p.a * r, vx) - delta, np.arctan2(vy - p.b * r, vx)


def axle_forces(p: BicycleParams, vx, vy, r, delta, ax, ay_load):
    """
    Tüm kuvvetler ve tanılar. Döner: dict.
    Teker dizileri [FL, FR, RL, RR]; Fx/Fy lastik eksenindedir (Fy hıza ters işaretli).
    """
    Fz, info = wheel_loads(p.mass, p.g, p.a, p.b, p.h_cg, p.track_f, p.track_r,
                           p.roll, ax, ay_load)
    af, ar = slip_angles(p, vx, vy, r, delta)

    drag = 0.5 * p.air_density * p.drag_coeff * p.frontal_area * vx * abs(vx)
    roll_res = p.rolling_resistance * p.mass * p.g
    Fx_req = p.mass * ax + drag + roll_res          # + = tahrik, - = fren
    if Fx_req < 0:
        Fx_axle = np.array([Fx_req * p.brake_bias_front, Fx_req * (1 - p.brake_bias_front)])
    else:
        Fx_axle = np.array([0.0, Fx_req])           # RWD
    # Eksen içi dağılım Fz ile orantılı (fren dengesi / LSD varsayımı).
    Fx = np.array([Fx_axle[0] * Fz[0] / (Fz[0] + Fz[1]), Fx_axle[0] * Fz[1] / (Fz[0] + Fz[1]),
                   Fx_axle[1] * Fz[2] / (Fz[2] + Fz[3]), Fx_axle[1] * Fz[3] / (Fz[2] + Fz[3])])
    tires = (p.tire_f, p.tire_f, p.tire_r, p.tire_r)
    alphas = (af, af, ar, ar)
    Fy = np.zeros(4)
    for i in range(4):
        Fxi = float(np.clip(Fx[i], -peak_longitudinal_force(Fz[i], tires[i]),
                            peak_longitudinal_force(Fz[i], tires[i])))
        Fx[i] = Fxi
        Fy[i] = lateral_force_with_fx(alphas[i], Fz[i], Fxi, tires[i])

    c, s = np.cos(delta), np.sin(delta)
    Fyb_f = (Fx[0] + Fx[1]) * s + (Fy[0] + Fy[1]) * c
    Fyb_r = Fy[2] + Fy[3]

    # Friction-circle kullanımı (aks bazında, 1.0 = limit)
    def util(i, j, tire):
        fxm = sum(peak_longitudinal_force(Fz[k], tire) for k in (i, j))
        fym = sum(peak_lateral_force(Fz[k], tire) for k in (i, j))
        return float(np.hypot((Fx[i] + Fx[j]) / fxm, (Fy[i] + Fy[j]) / fym))

    return dict(Fz=Fz, Fx=Fx, Fy=Fy, alpha_f=af, alpha_r=ar, Fyb_f=Fyb_f, Fyb_r=Fyb_r,
                util_f=util(0, 1, p.tire_f), util_r=util(2, 3, p.tire_r),
                dFz_f=info["dFz_f"], dFz_r=info["dFz_r"])


def derivatives(p: BicycleParams, state, vx, ax, delta):
    X, Y, psi, vy, r, ayf = state
    f = axle_forces(p, vx, vy, r, delta, ax, ayf)
    Fy_tot = f["Fyb_f"] + f["Fyb_r"]
    ay = Fy_tot / p.mass
    d = np.array([
        vx * np.cos(psi) - vy * np.sin(psi),
        vx * np.sin(psi) + vy * np.cos(psi),
        r,
        ay - vx * r,
        (p.a * f["Fyb_f"] - p.b * f["Fyb_r"]) / p.Iz,
        (ay - ayf) / p.roll_tau,
    ])
    return d


@dataclass
class Result:
    t: np.ndarray
    X: np.ndarray
    Y: np.ndarray
    psi: np.ndarray
    vx: np.ndarray
    vy: np.ndarray
    r: np.ndarray
    delta: np.ndarray
    ax: np.ndarray
    ay: np.ndarray
    s: np.ndarray            # kat edilen mesafe = ∫vx dt (tüm senaryolarda aynı)
    Fz: np.ndarray           # (N,4)
    Fx: np.ndarray
    Fy: np.ndarray
    alpha_f: np.ndarray
    alpha_r: np.ndarray
    util_f: np.ndarray
    util_r: np.ndarray
    dFz_f: np.ndarray
    dFz_r: np.ndarray


def simulate(p: BicycleParams, speed, steer_fn, dt=0.002, x0=(0.0, 0.0, 0.0)):
    """
    speed: .t_end, .v(t), .a(t) olan nesne. steer_fn(t, state, vx) -> delta [rad]
    (adım başında örneklenir, adım boyunca sabit tutulur). RK4.
    """
    n = int(np.floor(speed.t_end / dt)) + 1
    st = np.array([x0[0], x0[1], x0[2], 0.0, 0.0, 0.0])
    keys = ["t", "X", "Y", "psi", "vx", "vy", "r", "delta", "ax", "ay", "s"]
    rec = {k: np.zeros(n) for k in keys}
    Fz = np.zeros((n, 4)); Fx = np.zeros((n, 4)); Fy = np.zeros((n, 4))
    extra = {k: np.zeros(n) for k in ("alpha_f", "alpha_r", "util_f", "util_r", "dFz_f", "dFz_r")}
    s = 0.0
    for i in range(n):
        t = i * dt
        vx, ax = speed.v(t), speed.a(t)
        delta = steer_fn(t, st, vx)
        f = axle_forces(p, vx, st[3], st[4], delta, ax, st[5])
        rec["t"][i] = t; rec["X"][i], rec["Y"][i], rec["psi"][i] = st[0], st[1], st[2]
        rec["vx"][i] = vx; rec["vy"][i] = st[3]; rec["r"][i] = st[4]
        rec["delta"][i] = delta; rec["ax"][i] = ax
        rec["ay"][i] = (f["Fyb_f"] + f["Fyb_r"]) / p.mass; rec["s"][i] = s
        Fz[i], Fx[i], Fy[i] = f["Fz"], f["Fx"], f["Fy"]
        for k in extra:
            extra[k][i] = f[k]
        # RK4 (vx, ax, delta adım içinde sabit; vx adım ortası için yeniden örneklenir)
        vxm, axm = speed.v(t + dt / 2), speed.a(t + dt / 2)
        vx1, ax1 = speed.v(t + dt), speed.a(t + dt)
        k1 = derivatives(p, st, vx, ax, delta)
        k2 = derivatives(p, st + dt / 2 * k1, vxm, axm, delta)
        k3 = derivatives(p, st + dt / 2 * k2, vxm, axm, delta)
        k4 = derivatives(p, st + dt * k3, vx1, ax1, delta)
        st = st + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        s += 0.5 * (vx + vx1) * dt
    return Result(Fz=Fz, Fx=Fx, Fy=Fy, **rec, **extra)


# --------------------------------------------------------------------------
# Kararlı hal (sabit yarıçaplı daire) çözücüsü — doğrulama ve ay limiti için
# --------------------------------------------------------------------------
def steady_state(p: BicycleParams, v: float, R: float, guess=None):
    """
    vy_dot = 0, r_dot = 0 koşulunu (delta, beta) için Newton ile çözer.
    r = v/R, ay = v^2/R (sola dönüş). Çözüm yoksa None (limit aşıldı).
    Döner: dict(delta, beta, ay, util_f, util_r, alpha_f, alpha_r)
    """
    r = v / R
    ay = v * v / R

    def resid(x):
        delta, beta = x
        f = axle_forces(p, v, v * np.tan(beta), r, delta, 0.0, ay)
        # ax=0 iken sürükleme yine Fx talebi doğurur; kararlı-hal testinde
        # saf yanal davranışı görmek için sürükleme sıfırlanmış kopya kullan.
        return np.array([(f["Fyb_f"] + f["Fyb_r"]) / p.mass - v * r,
                         p.a * f["Fyb_f"] - p.b * f["Fyb_r"]]), f

    x = np.array([p.L / R, 0.0]) if guess is None else np.array(guess, float)
    for _ in range(60):
        F0, f = resid(x)
        if np.max(np.abs(F0)) < 1e-6 * p.mass:
            return dict(delta=x[0], beta=x[1], ay=ay, util_f=f["util_f"], util_r=f["util_r"],
                        alpha_f=f["alpha_f"], alpha_r=f["alpha_r"])
        J = np.zeros((2, 2))
        for j in range(2):
            dx = np.zeros(2); dx[j] = 1e-6
            J[:, j] = (resid(x + dx)[0] - F0) / 1e-6
        try:
            step = np.linalg.solve(J, -F0)
        except np.linalg.LinAlgError:
            return None
        x = x + np.clip(step, -0.05, 0.05)
    return None


def no_drag(p: BicycleParams) -> BicycleParams:
    return replace(p, drag_coeff=0.0, rolling_resistance=0.0)


def find_ay_limit(p: BicycleParams, R=60.0) -> float:
    """Sabit yarıçapta çözümün var olduğu en büyük ay [m/s^2] (bisection)."""
    q = no_drag(p)
    lo, hi = 5.0, 45.0
    guess = None
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        sol = steady_state(q, mid, R)
        if sol is None:
            hi = mid
        else:
            lo = mid
    return lo * lo / R
