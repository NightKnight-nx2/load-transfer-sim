"""
Grafikler: (a) pist üzerinde araç izi, (b) friction-circle kullanımı,
(c) sol/sağ yük transferi. Renkler: dataviz referans paleti, kategorik slot 1-3
(mavi/turuncu/aqua; ilk üçü all-pairs doğrulanmıştır). Ön/arka ayrımı renk
değil çizgi stiliyle yapılır (ön=düz, arka=kesikli); doğrudan etiketler ve
açık lejant vardır.
"""
from __future__ import annotations

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from matplotlib.patches import Polygon

from load_transfer_sim.scenario import SCENARIOS, lateral_deviation

SURFACE, INK, INK2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3de"
ROAD, ROAD_W = "#ecebe6", 4.0
G = 9.81

plt.rcParams.update({
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
    "text.color": INK, "axes.labelcolor": INK2, "xtick.color": INK2, "ytick.color": INK2,
    "axes.edgecolor": GRID, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.7,
    "axes.spines.top": False, "axes.spines.right": False, "font.size": 10,
    "axes.titlesize": 11, "axes.titleweight": "bold", "axes.titlelocation": "left",
    "legend.frameon": False, "lines.solid_capstyle": "round",
})


def _road(ax, tr):
    nx, ny = -np.sin(tr.heading), np.cos(tr.heading)
    L = np.column_stack([tr.x + ROAD_W * nx, tr.y + ROAD_W * ny])
    R = np.column_stack([tr.x - ROAD_W * nx, tr.y - ROAD_W * ny])
    ax.add_patch(Polygon(np.vstack([L, R[::-1]]), closed=True, fc=ROAD, ec="none", zorder=1))
    ax.plot(tr.x, tr.y, color="#b9b8b0", lw=0.8, ls=(0, (4, 4)), zorder=2)
    ax.set_aspect("equal"); ax.grid(False)


def _traces(ax, study, lw=1.8, ticks=False, win=None):
    for sc in SCENARIOS:
        r = study.results[sc.name]
        m = np.ones(len(r.t), bool) if win is None else (r.s >= win[0]) & (r.s <= win[1])
        ax.plot(r.X[m], r.Y[m], color=sc.color, lw=lw, zorder=4)
        for e in study.events[sc.name]:
            k = (r.t >= e.t0) & (r.t <= e.t1) & m
            if k.any():
                ax.plot(r.X[k], r.Y[k], color=sc.color, lw=lw + 3.2, alpha=0.35, zorder=3)
                i0 = np.flatnonzero(k)[0]
                ax.plot(r.X[i0], r.Y[i0], marker="v" if e.axle == "front" else "^",
                        ms=9, mfc=sc.color, mec=INK, mew=1.0, ls="none", zorder=6)
        if ticks:
            idx = np.flatnonzero(m)[::int(0.3 / (r.t[1] - r.t[0]))]
            for i in idx:
                ax.plot([r.X[i], r.X[i] + 2.6 * np.cos(r.psi[i])],
                        [r.Y[i], r.Y[i] + 2.6 * np.sin(r.psi[i])],
                        color=sc.color, lw=1.1, zorder=5, alpha=0.9)
                ax.plot(r.X[i], r.Y[i], "o", ms=2.3, color=sc.color, zorder=5)


def _legend_handles():
    h = [Line2D([0], [0], color=s.color, lw=2.4, label=s.label) for s in SCENARIOS]
    h += [Line2D([0], [0], marker="v", ls="none", mfc="#cfcfcf", mec=INK, ms=8,
                 label="ön aks limite girdi → understeer"),
          Line2D([0], [0], marker="^", ls="none", mfc="#cfcfcf", mec=INK, ms=8,
                 label="arka aks limite girdi → oversteer")]
    return h


def _bounds(tr, s0, s1, pad):
    m = (tr.s >= s0) & (tr.s <= s1)
    return (tr.x[m].min() - pad, tr.x[m].max() + pad, tr.y[m].min() - pad, tr.y[m].max() + pad)


def plot_traces(study, path):
    tr, cs = study.track, study.corners
    fig = plt.figure(figsize=(13, 15.5))
    gs = fig.add_gridspec(3, 2, height_ratios=[3.1, 2.7, 1.45], hspace=0.28, wspace=0.12)
    ax = fig.add_subplot(gs[0, :])
    _road(ax, tr); _traces(ax, study, lw=1.6)
    for c in cs:
        i = np.argmin(np.abs(tr.s - 0.5 * (c.s0 + c.s1)))
        nx, ny = -np.sin(tr.heading[i]), np.cos(tr.heading[i])
        sgn = -1 if c.direction == "L" else 1
        ax.text(tr.x[i] + sgn * 8 * nx, tr.y[i] + sgn * 8 * ny, c.name.split()[0], ha="center",
                va="center", fontsize=10, color=INK2, fontweight="bold", zorder=7)
    ax.plot(tr.x[0], tr.y[0], "o", color=INK, ms=6, zorder=7)
    ax.text(tr.x[0] + 3, tr.y[0] - 9, "start", fontsize=9, color=INK2)
    ax.set_title("Pist üzerinde araç izi — aynı sürücü, aynı hız profili; yalnızca K_f/K_r farklı")
    ax.legend(handles=_legend_handles(), loc="upper right", fontsize=9, ncol=1)
    ax.set_xlabel("X [m]"); ax.set_ylabel("Y [m]")

    sw = [c for c in cs if c.kind == "sweeper"][0]
    sb = [c for c in cs if c.kind == "S-bend"]
    zooms = [("T1 braking sweeper — zoom (çizgi ucu = araç yönü, 0.3 s aralık)", sw.s0 - 12, sw.s1 + 12, 6),
             ("S-bend — zoom", sb[0].s0 - 12, sb[-1].s1 + 12, 6)]
    for k, (title, s0, s1, pad) in enumerate(zooms):
        a = fig.add_subplot(gs[1, k])
        _road(a, tr); _traces(a, study, lw=1.9, ticks=True, win=(s0 - 6, s1 + 6))
        x0, x1, y0, y1 = _bounds(tr, s0, s1, pad)
        a.set_xlim(x0, x1); a.set_ylim(y0, y1)
        a.set_title(title, fontsize=10); a.set_xlabel("X [m]")
        if k == 0:
            a.set_ylabel("Y [m]")

    a = fig.add_subplot(gs[2, :])
    for c in cs:
        a.axvspan(c.s0, c.s1, color="#000000", alpha=0.05, lw=0)
        a.text(0.5 * (c.s0 + c.s1), 0.97, c.name.split()[0], transform=a.get_xaxis_transform(),
               ha="center", va="top", fontsize=9, color=INK2)
    a.axhline(0, color="#b9b8b0", lw=1)
    for sc in SCENARIOS:
        r = study.results[sc.name]
        dev = lateral_deviation(r, tr)
        a.plot(r.s, dev, color=sc.color, lw=1.8)
    a.set_xlabel("kat edilen mesafe [m]")
    a.set_ylabel("yanal sapma [m]\n(+ sol, − sağ)")
    a.set_title("Yol eksenine göre yanal sapma — sola dönüşte negatif = dışa açıldı (understeer), pozitif = içe kesti (oversteer)",
                fontsize=10)
    a.set_xlim(0, tr.s[-1] + 2)
    fig.savefig(path, dpi=140, bbox_inches="tight")
    plt.close(fig)


def _mark_corners(ax, cs, label=False):
    for c in cs:
        ax.axvspan(c.s0, c.s1, color="#000000", alpha=0.05, lw=0)
        if label:
            ax.text(0.5 * (c.s0 + c.s1), 1.02, c.name.split()[0], transform=ax.get_xaxis_transform(),
                    ha="center", va="bottom", fontsize=9, color=INK2)


def plot_grip(study, path):
    cs = study.corners
    fig, axs = plt.subplots(4, 1, figsize=(13, 11.5), sharex=True,
                            gridspec_kw=dict(height_ratios=[1.0, 1.6, 1.6, 1.6], hspace=0.18))
    r0 = study.results["balanced"]
    axs[0].plot(r0.s, r0.vx * 3.6, color=INK2, lw=1.8)
    axs[0].set_ylabel("hız [km/h]"); _mark_corners(axs[0], cs, label=True)
    axs[0].set_title("Ortak hız profili ve virajlar", fontsize=10)
    for ax, sc in zip(axs[1:], SCENARIOS):
        r = study.results[sc.name]
        _mark_corners(ax, cs)
        ax.axhline(100, color=INK, lw=1.0)
        ax.text(2, 100.8, "kayma limiti = %100", fontsize=8, color=INK2, va="bottom")
        ax.plot(r.s, 100 * r.util_f, color=sc.color, lw=1.9, ls="-")
        ax.plot(r.s, 100 * r.util_r, color=sc.color, lw=1.9, ls=(0, (5, 2.5)))
        for e in study.events[sc.name]:
            ax.axvspan(e.s0, max(e.s1, e.s0 + 1.0), ymin=0.0, ymax=1.0, color=sc.color, alpha=0.18, lw=0)
            ax.plot(e.s0, 100 * (r.util_f if e.axle == "front" else r.util_r)[np.searchsorted(r.t, e.t0)],
                    marker="v" if e.axle == "front" else "^", ms=9, mfc=sc.color, mec=INK, mew=1.0,
                    ls="none", zorder=6)
        ax.set_ylim(0, 112); ax.set_ylabel("grip kullanımı [%]")
        ax.set_title(sc.label, fontsize=10, color=INK)
        ax.text(r.s[-1] + 2, 100 * r.util_f[-1], "ön", fontsize=9, color=INK2, va="center")
        ax.text(r.s[-1] + 2, 100 * r.util_r[-1] - 4, "arka", fontsize=9, color=INK2, va="center")
    axs[-1].set_xlabel("kat edilen mesafe [m]")
    axs[-1].set_xlim(0, r0.s[-1] + 14)
    h = [Line2D([0], [0], color=INK2, lw=2, label="ön aks"),
         Line2D([0], [0], color=INK2, lw=2, ls=(0, (5, 2.5)), label="arka aks"),
         Line2D([0], [0], marker="v", ls="none", mfc="#cfcfcf", mec=INK, ms=8, label="ön limit başlangıcı (understeer)"),
         Line2D([0], [0], marker="^", ls="none", mfc="#cfcfcf", mec=INK, ms=8, label="arka limit başlangıcı (oversteer)")]
    fig.legend(handles=h, loc="lower center", ncol=4, fontsize=9, bbox_to_anchor=(0.5, 0.0))
    fig.suptitle("Combined grip kullanımı — friction circle, √((Fx/Fx,max)² + (Fy/Fy,max)²) aks bazında",
                 x=0.125, ha="left", fontsize=12, fontweight="bold", y=0.95)
    fig.savefig(path, dpi=140, bbox_inches="tight")
    plt.close(fig)


def plot_load_transfer(study, path):
    cs = study.corners
    fig = plt.figure(figsize=(13, 12.5))
    gs = fig.add_gridspec(4, 2, height_ratios=[1.0, 1.5, 1.5, 1.5], hspace=0.3, wspace=0.16)
    a0 = fig.add_subplot(gs[0, :])
    _mark_corners(a0, cs, label=True)
    for sc in SCENARIOS:
        r = study.results[sc.name]
        a0.plot(r.s, r.ay / G, color=sc.color, lw=1.7)
    a0.axhline(0, color="#b9b8b0", lw=1)
    a0.set_ylabel("yanal ivme [g]\n(+ sola)"); a0.set_title("Yanal ivme (yük transferini süren girdi)", fontsize=10)
    first = None
    for row, sc in enumerate(SCENARIOS, 1):
        r = study.results[sc.name]
        aL = fig.add_subplot(gs[row, 0], sharex=first or a0)
        aR = fig.add_subplot(gs[row, 1], sharex=a0)
        first = first or aL
        _mark_corners(aL, cs); _mark_corners(aR, cs)
        aL.axhline(0, color="#b9b8b0", lw=1)
        aL.plot(r.s, r.dFz_f, color=sc.color, lw=1.9, label="ön aks")
        aL.plot(r.s, r.dFz_r, color=sc.color, lw=1.9, ls=(0, (5, 2.5)), label="arka aks")
        aL.legend(loc="upper right", fontsize=8, ncol=2)
        aL.set_ylabel("sol→sağ transfer [N]")
        aL.set_title(f"{sc.label} — aks başına yanal transfer", fontsize=10, color=INK)
        names = ["FL", "FR", "RL", "RR"]
        styles = [(0, (1, 1.5)), "-", (0, (1, 1.5)), "-"]
        for k in range(4):
            ls = ["-", "-", (0, (5, 2.5)), (0, (5, 2.5))][k]
            aR.plot(r.s, r.Fz[:, k], color=sc.color, lw=1.2 if k % 2 == 0 else 2.0, ls=ls, label=names[k])
        aR.legend(loc="upper right", fontsize=8, ncol=4)
        aR.set_title("tekerlek normal yükleri (ince=sol, kalın=sağ; düz=ön, kesikli=arka)", fontsize=10)
        aR.set_ylabel("Fz [N]"); aR.set_ylim(0, 8200); aL.set_ylim(-3600, 4300)
        for a in (aL, aR):
            a.set_xlim(0, r.s[-1])
    for a in fig.axes[-2:]:
        a.set_xlabel("kat edilen mesafe [m]")
    fig.suptitle("Sol/sağ yük transferi — toplam transfer K_f/K_r oranıyla ön/arka akslara paylaşılır",
                 x=0.125, ha="left", fontsize=12, fontweight="bold", y=0.945)
    fig.savefig(path, dpi=140, bbox_inches="tight")
    plt.close(fig)
