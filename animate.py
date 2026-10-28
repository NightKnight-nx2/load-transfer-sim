"""
Görsel simülasyon: aynı Python simülasyonunu (3 senaryo) çalıştırır, 30 Hz'e
örnekleyip tek dosyalık bir HTML'e gömer. Sunucu gerekmez; dosyayı çift
tıklayıp tarayıcıda aç.

    python -m load_transfer_sim.animate        # -> output/simulasyon.html
"""
from __future__ import annotations

import json
import os
import sys

import numpy as np

from load_transfer_sim.scenario import SCENARIOS, make_study, run_scenario

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "output")


def build_payload(fps=30):
    study = make_study()
    for sc in SCENARIOS:
        run_scenario(study, sc)
    r0 = study.results["balanced"]
    n = int(r0.t[-1] * fps) + 1
    tt = np.arange(n) / fps

    def smp(res, arr, d):
        return np.round(np.interp(tt, res.t, arr), d).tolist()

    tr = study.track
    idx = np.arange(0, len(tr.s), 4)               # ~1 m aralık
    payload = dict(
        fps=fps, n=n, T=float(tt[-1]),
        s=smp(r0, r0.s, 1),
        track=dict(s=np.round(tr.s[idx], 1).tolist(), x=np.round(tr.x[idx], 2).tolist(),
                   y=np.round(tr.y[idx], 2).tolist(), h=np.round(tr.heading[idx], 4).tolist()),
        corners=[dict(name=c.name, kind=c.kind, dir=c.direction, s0=c.s0, s1=c.s1) for c in study.corners],
        car=dict(a=study.base.a, b=study.base.b, track=study.base.track_f),
        scen=[],
    )
    for sc in SCENARIOS:
        r = study.results[sc.name]
        ev = study.events[sc.name]
        f_lim = np.zeros(n, int); r_lim = np.zeros(n, int)
        for e in ev:
            m = (tt >= e.t0) & (tt <= e.t1)
            (f_lim if e.axle == "front" else r_lim)[m] = 1
        payload["scen"].append(dict(
            name=sc.name, label=sc.label, share=sc.front_share,
            X=smp(r, r.X, 2), Y=smp(r, r.Y, 2), psi=smp(r, r.psi, 3), delta=smp(r, r.delta, 3),
            beta=smp(r, np.arctan2(r.vy, r.vx), 3), v=smp(r, r.vx, 2), ay=smp(r, r.ay / 9.81, 3),
            uf=smp(r, r.util_f, 3), ur=smp(r, r.util_r, 3),
            fz=[smp(r, r.Fz[:, k], 0) for k in range(4)],
            dff=smp(r, r.dFz_f, 0), dfr=smp(r, r.dFz_r, 0),
            fl=f_lim.tolist(), rl=r_lim.tolist(),
            events=[dict(axle=e.axle, kind=e.kind, corner=e.corner, t0=round(e.t0, 2), t1=round(e.t1, 2))
                    for e in ev],
        ))
    return payload


def main():
    os.makedirs(OUT, exist_ok=True)
    payload = build_payload()
    with open(os.path.join(HERE, "viewer_template.html"), encoding="utf-8") as f:
        html = f.read()
    data = json.dumps(payload, separators=(",", ":"), ensure_ascii=False)
    assert "__DATA__" in html
    html = html.replace("__DATA__", data.replace("</", "<\\/"))
    path = os.path.join(OUT, "simulasyon.html")
    with open(path, "w", encoding="utf-8") as f:
        f.write(html)
    print(f"yazıldı: {path}  ({os.path.getsize(path)/1024:.0f} KB, {payload['n']} kare)")


if __name__ == "__main__":
    sys.exit(main())
