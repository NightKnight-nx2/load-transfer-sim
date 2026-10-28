"""
Balanced senaryosunun doğrulaması (diğer iki senaryodan ÖNCE koşar).
Analitik beklentiler:
  * a=b, aynı lastik, K_f=K_r  =>  araç simetrik => kararlı halde Fyf=Fyr =>
    alpha_f = alpha_r => delta_ss = L/R (nötr, understeer gradyanı 0) ve
    ön/arka grip kullanımı eşit — limite kadar, her hızda.
  * Fz toplamı m*g; transfer momenti m*ay*h; frende yük öne gider.
  * Pacejka: Fy(0)=0, tek, tepe = mu(Fz)*Fz, yük hassasiyeti içbükey.
"""
from __future__ import annotations

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from load_transfer_sim import tire_model as tm
from load_transfer_sim.bicycle_model import find_ay_limit, no_drag, steady_state
from load_transfer_sim.load_transfer import RollStiffness, wheel_loads
from load_transfer_sim.scenario import SCENARIOS, lateral_deviation
from load_transfer_sim.visualize import GRID, INK, INK2

G = 9.81


def handling_sweep(p, R=60.0, n=26):
    q = no_drag(p)
    lim = find_ay_limit(p, R)
    out = []
    for ay in np.linspace(0.5, 0.985 * lim, n):
        v = np.sqrt(ay * R)
        s = steady_state(q, v, R)
        if s is not None:
            out.append((ay / G, (s["delta"] - p.L / R) * 180 / np.pi, s["util_f"], s["util_r"]))
    return np.array(out), lim / G


def validate_balanced(study, res, fig_path):
    p = study.base
    checks = []

    def chk(name, ok, detail=""):
        checks.append((name, bool(ok), detail))

    # ---- lastik
    t = p.tire_f
    ap = tm.peak_slip_angle(t)
    Fz0 = 3433.0
    chk("Fy(α=0)=0 ve tek fonksiyon", abs(tm.pure_lateral_force(0.0, Fz0, t)) < 1e-9 and
        abs(tm.pure_lateral_force(0.1, Fz0, t) + tm.pure_lateral_force(-0.1, Fz0, t)) < 1e-6)
    chk("tepe Fy = mu(Fz)*Fz (α_pik=%.1f°)" % np.degrees(ap),
        abs(abs(tm.pure_lateral_force(ap, Fz0, t)) - tm.peak_lateral_force(Fz0, t)) < 1.0)
    d_lo = tm.peak_lateral_force(2000, t) - tm.peak_lateral_force(1000, t)
    d_hi = tm.peak_lateral_force(6000, t) - tm.peak_lateral_force(5000, t)
    chk("yük hassasiyeti: +1000 N ek yük, çok yüklü tekerde daha az Fy kazandırır",
        d_hi < d_lo, f"+1000 N: düşük yükte +{d_lo:.0f} N, yüksek yükte +{d_hi:.0f} N")

    # ---- yük transferi
    rs = RollStiffness()
    Fz, info = wheel_loads(p.mass, G, p.a, p.b, p.h_cg, p.track_f, p.track_r, rs, 0.0, G)
    chk("ΣFz = m·g", abs(Fz.sum() - p.mass * G) < 1e-6, f"{Fz.sum():.1f} N")
    mom = info["dFz_f"] * p.track_f + info["dFz_r"] * p.track_r
    chk("transfer momenti = m·ay·h", abs(mom - p.mass * G * p.h_cg) < 1e-6)
    chk("balanced: ön transfer = arka transfer", abs(info["dFz_f"] - info["dFz_r"]) < 1e-9,
        f"{info['dFz_f']:.0f} N / {info['dFz_r']:.0f} N")
    chk("sola dönüşte (ay>0) sağ tekerler yüklenir", Fz[1] > Fz[0] and Fz[3] > Fz[2])
    Fzf, _ = wheel_loads(p.mass, G, p.a, p.b, p.h_cg, p.track_f, p.track_r,
                         RollStiffness.from_share(0.7), 0.0, G)[1], None
    chk("front-stiff: ön aks payı %70", abs(Fzf["dFz_f"] * p.track_f / (p.mass * G * p.h_cg) - 0.7) < 1e-9)
    Fb, _ = wheel_loads(p.mass, G, p.a, p.b, p.h_cg, p.track_f, p.track_r, rs, -6.0, 0.0)
    chk("frenleme yükü öne taşır (m·|ax|·h/L)",
        abs((Fb[0] + Fb[1]) - (p.mass * G / 2 + p.mass * 6.0 * p.h_cg / p.L)) < 1e-6)

    # ---- kararlı hal
    sweeps = {}
    for sc in SCENARIOS:
        pp = p.with_roll_share(sc.front_share)
        sweeps[sc.name] = handling_sweep(pp)
    sb, lim_b = sweeps["balanced"]
    chk("balanced nötr: |δ_ss − L/R| < 0.05° (limite kadar; kalan = cosδ geometrisi)", np.abs(sb[:, 1]).max() < 0.05,
        f"maks {np.abs(sb[:, 1]).max():.4f}°")
    chk("balanced: ön/arka grip kullanımı eşit (<0.5 puan)", np.abs(sb[:, 2] - sb[:, 3]).max() < 0.005)
    sf, lim_f = sweeps["front_stiff"]; sr, lim_r = sweeps["rear_stiff"]
    chk("front-stiff understeer (δ−L/R > 0), rear-stiff oversteer (< 0) yüksek ay'de",
        sf[-1, 1] > 0.2 and sr[-1, 1] < -0.2, f"{sf[-1,1]:+.2f}° / {sr[-1,1]:+.2f}°")
    chk("balanced en yüksek ay limitine sahip", lim_b > lim_f and lim_b > lim_r,
        f"{lim_b:.3f} g > {lim_f:.3f} g (front-stiff), {lim_r:.3f} g (rear-stiff)")

    # ---- dinamik koşu
    dev = lateral_deviation(res, study.track)
    chk("balanced pisti izler: |yanal sapma| < 1.5 m", np.abs(dev).max() < 1.5, f"maks {np.abs(dev).max():.2f} m")
    chk("ΣFz = m·g tüm koşu boyunca", np.abs(res.Fz.sum(1) - p.mass * G).max() < 1e-6)
    mask = np.abs(res.ay) > 0.3 * G
    dd = np.median(np.abs(res.util_f - res.util_r)[mask])
    chk("dinamikte ön/arka kullanım dengeli (medyan fark < %3, ay>0.3 g)", dd < 0.03, f"{100*dd:.1f} puan")
    chk("balanced limiti aşmıyor (olay yok)", len(study.events["balanced"]) == 0,
        f"tepe kullanım ön %{100*res.util_f.max():.1f} / arka %{100*res.util_r.max():.1f}")

    print("\nBalanced doğrulaması:")
    for n, ok, d in checks:
        print(f"  [{'PASS' if ok else 'FAIL'}] {n}" + (f"  — {d}" if d else ""))
    allok = all(c[1] for c in checks)
    print("  =>", "TÜMÜ GEÇTİ" if allok else "BAŞARISIZ")

    # ---- şekil
    from load_transfer_sim.visualize import plt as _plt  # tema rcParams'ı yüklensin
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(12.5, 4.8))
    for sc in SCENARIOS:
        d, _ = sweeps[sc.name]
        a1.plot(d[:, 0], d[:, 1], color=sc.color, lw=2.0)
        a1.text(d[-1, 0] + 0.01, d[-1, 1], sc.name.replace("_", "-"), fontsize=9, color=INK2, va="center")
    a1.axhline(0, color=INK, lw=1)
    a1.set_xlabel("yanal ivme [g]"); a1.set_ylabel("δ − L/R [°]")
    a1.set_title("Handling diyagramı (R=60 m kararlı hal)\n+ understeer, − oversteer, 0 = nötr", fontsize=10)
    a1.set_xlim(0, 1.12)
    Fz_ax = np.linspace(500, 7000, 200)
    Dy = np.array([tm.peak_lateral_force(z, p.tire_f) for z in Fz_ax])
    a2.plot(Fz_ax, Dy, color=INK, lw=2.0, label="tepe Fy (Pacejka, yük hassas)")
    a2.plot(Fz_ax, Dy[0] / Fz_ax[0] * Fz_ax, color=INK2, lw=1.2, ls=(0, (5, 2.5)),
            label="orantılı olsaydı (sabit μ)")
    a2.set_xlabel("tekerlek normal yükü Fz [N]"); a2.set_ylabel("tepe yanal kuvvet [N]")
    a2.set_title("Yük hassasiyeti: fazla yüklenen teker orantısız az tutunma kazanır", fontsize=10)
    a2.legend(fontsize=9, loc="upper left")
    fig.savefig(fig_path, dpi=140, bbox_inches="tight")
    plt.close(fig)
    return allok
