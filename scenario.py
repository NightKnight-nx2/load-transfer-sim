"""
Pist, hız profili, sürücü girdisi, üç senaryo ve limit tespiti.

Adil karşılaştırma için tasarım:
  1. HIZ: v(s) bir kez hesaplanır (g-g/friction-circle sınırlı iki geçişli
     profil, balanced aracın kararlı-hal ay limitine göre). Üç senaryo da
     AYNI v(t)'yi sürer.
  2. SÜRÜCÜ: üç senaryoda da AYNI kontrolcü (feed-forward L*k + Stanley)
     ve aynı kazançlar. Fark yalnızca K_f/K_r dağılımından gelir.
     (Aynı direksiyon zamanlamasını açık döngü oynatmak denendi: yanal
     dinamik kararsız olduğundan oversteer eden araç tek viraj sonra
     tamamen savruluyor ve karşılaştırma anlamsızlaşıyor; bu yüzden sürücü
     kapalı döngüdür.)
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from load_transfer_sim.bicycle_model import (
    BicycleParams, Result, find_ay_limit, no_drag, simulate,
)
from load_transfer_sim.tire_model import peak_slip_angle
from load_transfer_sim.raceline import Raceline, build_oval_with_chicane

G = 9.81


# ------------------------------------------------------------------ pist
@dataclass
class Corner:
    name: str
    kind: str        # "sweeper" | "S-bend"
    direction: str   # "L" | "R"
    s0: float
    s1: float


def build_track(ds=0.25, smooth_m=8.0):
    """Sistem 7'nin oval+chicane pisti (geniş sweeper'lar, 40° S-bend). Eğrilik
    kutu filtresiyle yumuşatılıp yeniden entegre edilir (klotoid benzeri geçiş)."""
    raw = build_oval_with_chicane(straight1=90.0, straight2=45.0, straight3=45.0,
                                  straight4=25.0, oval_radius=34.0, chicane_radius=18.0,
                                  chicane_angle_deg=40.0, ds=ds)
    L = raw.s[-1]
    s = np.arange(0.0, L, ds)
    kap_raw = np.interp(s, raw.s, raw.curvature)

    # köşe segmentleri (ham eğrilikten): işaret değişimi/sıfır sınır olur
    corners = []
    sig = np.where(np.abs(kap_raw) > 1e-6, np.sign(kap_raw), 0.0)
    edges = np.flatnonzero(np.diff(sig) != 0) + 1
    bounds = np.concatenate([[0], edges, [len(s)]])
    k = 0
    for a, b in zip(bounds[:-1], bounds[1:]):
        if sig[a] == 0:
            continue
        k += 1
        R = 1.0 / np.median(np.abs(kap_raw[a:b]))
        kind = "sweeper" if R > 25 else "S-bend"
        d = "L" if sig[a] > 0 else "R"
        corners.append(Corner(f"T{k} {kind} ({d})", kind, d, float(s[a]), float(s[b - 1])))

    w = max(int(round(smooth_m / ds)), 1)
    pad = np.pad(kap_raw, (w, w), mode="edge")
    kap = np.convolve(pad, np.ones(w) / w, mode="same")[w:-w]
    head = np.concatenate([[0.0], np.cumsum(0.5 * (kap[1:] + kap[:-1]) * ds)])
    hm = 0.5 * (head[1:] + head[:-1])
    x = np.concatenate([[0.0], np.cumsum(ds * np.cos(hm))])
    y = np.concatenate([[0.0], np.cumsum(ds * np.sin(hm))])
    return Raceline(s=s, x=x, y=y, heading=head, curvature=kap, ds=ds), corners


# ------------------------------------------------------------- hız profili
@dataclass
class SpeedProfile:
    t_grid: np.ndarray
    v_grid: np.ndarray
    a_grid: np.ndarray
    s_grid: np.ndarray
    t_end: float
    ay_limit: float
    grip_use: float

    def v(self, t):
        return float(np.interp(t, self.t_grid, self.v_grid))

    def a(self, t):
        return float(np.interp(t, self.t_grid, self.a_grid))


def build_speed_profile(track: Raceline, ay_limit, grip_use=0.93, ax_brake=6.0,
                        ax_accel=3.0, v0=14.0, v_max=38.0) -> SpeedProfile:
    """g-g (friction-ellipse) sınırlı iki geçişli profil: viraj hızı sqrt(grip_use*ay_lim/|k|),
    kalan boylamsal kapasite ax = ax_max*sqrt(1-(ay/ay_lim_use)^2)."""
    ds = track.ds
    kap = np.abs(track.curvature)
    ay_use = grip_use * ay_limit
    v = np.minimum(np.sqrt(ay_use / np.maximum(kap, 1e-6)), v_max)
    v[0] = min(v[0], v0)
    n = len(v)

    def ell(i, vi):
        return np.sqrt(max(1.0 - (vi * vi * kap[i] / ay_use) ** 2, 0.0))

    for i in range(n - 2, -1, -1):
        v[i] = min(v[i], np.sqrt(v[i + 1] ** 2 + 2 * ax_brake * ell(i + 1, v[i + 1]) * ds))
    v[0] = min(v[0], v0)
    for i in range(1, n):
        v[i] = min(v[i], np.sqrt(v[i - 1] ** 2 + 2 * ax_accel * ell(i - 1, v[i - 1]) * ds))

    vm = 0.5 * (v[1:] + v[:-1])
    t = np.concatenate([[0.0], np.cumsum(ds / vm)])
    # ax = dv/dt, 0.25 s'lik kayan ortalama ile yumuşat
    a = np.gradient(v, t)
    tt = np.arange(0.0, t[-1], 0.01)
    vt, at = np.interp(tt, t, v), np.interp(tt, t, a)
    k = max(int(0.25 / 0.01), 1)
    at = np.convolve(np.pad(at, (k, k), mode="edge"), np.ones(k) / k, mode="same")[k:-k]
    st = np.concatenate([[0.0], np.cumsum(0.5 * (vt[1:] + vt[:-1]) * 0.01)])
    return SpeedProfile(tt, vt, at, st, float(tt[-1]), ay_limit, grip_use)


# ----------------------------------------------------------------- sürücü
class StanleyDriver:
    """Referans (balanced) direksiyonu üretmek için: feed-forward L*k + Stanley geri beslemesi."""
    def __init__(self, track: Raceline, L, k_e=1.2, k_psi=1.0, delta_max=0.6):
        self.tr, self.L, self.k_e, self.k_psi, self.dm = track, L, k_e, k_psi, delta_max
        self.s_guess = 0.0

    def __call__(self, t, state, vx):
        X, Y, psi = state[0], state[1], state[2]
        s_p = self.tr.nearest_s(X, Y, self.s_guess, 12.0)
        self.s_guess = s_p
        xp, yp, hp, kp = self.tr.point_at_s(s_p)
        e_y = -np.sin(hp) * (X - xp) + np.cos(hp) * (Y - yp)       # sol = +
        e_psi = (hp - psi + np.pi) % (2 * np.pi) - np.pi
        d = self.L * kp + self.k_psi * e_psi - np.arctan(self.k_e * e_y / (vx + 2.0))
        return float(np.clip(d, -self.dm, self.dm))


class ReplaySteer:
    def __init__(self, t, delta):
        self.t, self.d = t, delta

    def __call__(self, t, state, vx):
        return float(np.interp(t, self.t, self.d))


# ---------------------------------------------------------------- senaryo
@dataclass
class Scenario:
    name: str
    label: str
    front_share: float     # K_f/(K_f+K_r)
    color: str


# kategorik slot 1-3 (dataviz referans paleti, light)
SCENARIOS = [
    Scenario("balanced", "Balanced (K_f:K_r = 50:50)", 0.50, "#2a78d6"),
    Scenario("front_stiff", "Front-stiff (K_f:K_r = 70:30)", 0.70, "#eb6834"),
    Scenario("rear_stiff", "Rear-stiff (K_f:K_r = 36:64)", 0.36, "#1baf7a"),
]


@dataclass
class LimitEvent:
    scenario: str
    axle: str            # "front" | "rear"
    kind: str            # "understeer" | "oversteer"
    corner: str
    t0: float
    t1: float
    s0: float
    s1: float
    X0: float
    Y0: float
    peak_util: float
    peak_alpha_ratio: float


def detect_limit_events(name, res: Result, tire_f, tire_r, corners, util_hi=0.985,
                        min_dur=0.10, merge_gap=0.15, margin_m=8.0):
    """Aks, util >= util_hi VEYA |alpha| >= alpha_peak ise limitte. Kısa boşluklar birleştirilir,
    0.1 s'den kısa olaylar atılır. Olay, s-aralığı (±margin) ile örtüşen virajla etiketlenir."""
    dt = res.t[1] - res.t[0]
    events = []
    for axle, util, alpha, tire in (("front", res.util_f, res.alpha_f, tire_f),
                                    ("rear", res.util_r, res.alpha_r, tire_r)):
        ap = peak_slip_angle(tire)
        on = (util >= util_hi) | (np.abs(alpha) >= ap)
        idx = np.flatnonzero(on)
        if idx.size == 0:
            continue
        groups, start, prev = [], idx[0], idx[0]
        for i in idx[1:]:
            if (i - prev) * dt > merge_gap:
                groups.append((start, prev)); start = i
            prev = i
        groups.append((start, prev))
        for a, b in groups:
            if (b - a) * dt < min_dur:
                continue
            sa, sb = res.s[a], res.s[b]
            best, dmin = "straight", margin_m
            for c in corners:                      # olayın BAŞLANGICINA göre etiket
                d = 0.0 if c.s0 <= sa <= c.s1 else min(abs(sa - c.s0), abs(sa - c.s1))
                if d < dmin or (d == 0.0):
                    best, dmin = c.name, d
                    if d == 0.0:
                        break
            events.append(LimitEvent(name, axle, "understeer" if axle == "front" else "oversteer",
                                     best, res.t[a], res.t[b], sa, sb, res.X[a], res.Y[a],
                                     float(util[a:b + 1].max()),
                                     float(np.abs(alpha[a:b + 1]).max() / ap)))
    events.sort(key=lambda e: e.t0)
    return events


def lateral_deviation(res: Result, track: Raceline) -> np.ndarray:
    """Her örnek için yola en yakın nokta ile işaretli yanal sapma [m] (sol = +)."""
    dev = np.zeros(len(res.t))
    sg = 0.0
    for i in range(len(res.t)):
        sg = track.nearest_s(res.X[i], res.Y[i], sg, 25.0)
        xp, yp, hp, _ = track.point_at_s(sg)
        dev[i] = -np.sin(hp) * (res.X[i] - xp) + np.cos(hp) * (res.Y[i] - yp)
    return dev


@dataclass
class Study:
    track: Raceline
    corners: list
    base: BicycleParams
    speed: SpeedProfile
    results: dict = field(default_factory=dict)
    events: dict = field(default_factory=dict)


def make_study(grip_use=0.90, ax_brake=5.0, **speed_kw) -> Study:
    base = BicycleParams.from_sistem7(a=1.35, b=1.35, brake_bias_front=0.65)
    track, corners = build_track()
    ay_lim = find_ay_limit(base)                      # balanced aracın ay limiti
    speed = build_speed_profile(track, ay_lim, grip_use=grip_use, ax_brake=ax_brake, **speed_kw)
    return Study(track, corners, base, speed)


def run_scenario(study: Study, sc: Scenario, dt=0.002) -> Result:
    p = study.base.with_roll_share(sc.front_share)
    res = simulate(p, study.speed, StanleyDriver(study.track, p.L), dt=dt)
    study.results[sc.name] = res
    study.events[sc.name] = detect_limit_events(sc.name, res, p.tire_f, p.tire_r, study.corners)
    return res
