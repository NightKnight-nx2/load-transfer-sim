"""
Kullanım (SomeSortOfSimulations klasöründen):
    python -m load_transfer_sim.run balanced   # 1. aşama: yalnızca balanced + doğrulama
    python -m load_transfer_sim.run all        # 2. aşama: üç senaryo + grafikler
"""
import csv
import os
import sys

import numpy as np

from load_transfer_sim.scenario import SCENARIOS, make_study, run_scenario, lateral_deviation
from load_transfer_sim.validate_balanced import validate_balanced

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")


def write_events(study, path):
    rows = []
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["senaryo", "aks", "tur", "viraj", "t0_s", "t1_s", "s0_m", "s1_m",
                    "tepe_kullanim_pct", "tepe_alpha/alpha_pik"])
        for sc in SCENARIOS:
            for e in study.events.get(sc.name, []):
                w.writerow([e.scenario, e.axle, e.kind, e.corner, f"{e.t0:.2f}", f"{e.t1:.2f}",
                            f"{e.s0:.1f}", f"{e.s1:.1f}", f"{100*e.peak_util:.1f}", f"{e.peak_alpha_ratio:.2f}"])
    return rows


def write_csv(res, path):
    cols = ["t", "s", "X", "Y", "psi", "vx", "vy", "r", "delta", "ax", "ay",
            "alpha_f", "alpha_r", "util_f", "util_r", "dFz_f", "dFz_r"]
    data = [getattr(res, c) for c in cols]
    for k, n in enumerate(["FL", "FR", "RL", "RR"]):
        cols.append("Fz_" + n); data.append(res.Fz[:, k])
    np.savetxt(path, np.column_stack(data), delimiter=",", header=",".join(cols), comments="", fmt="%.5f")


def print_events(study):
    print("\nLimit olayları (aks limite girdi):")
    for sc in SCENARIOS:
        ev = study.events.get(sc.name)
        if ev is None:
            continue
        print(f"  [{sc.name}]" + ("  — limit aşılmadı" if not ev else ""))
        for e in ev:
            print(f"    {e.kind:10s} {e.corner:18s} t={e.t0:5.2f}-{e.t1:5.2f}s  s={e.s0:5.1f}-{e.s1:5.1f}m  "
                  f"tepe kullanım %{100*e.peak_util:5.1f}  α/α_pik {e.peak_alpha_ratio:.2f}")


def main(stage="all"):
    os.makedirs(OUT, exist_ok=True)
    study = make_study()
    print(f"Hız profili: ay_limit(balanced)={study.speed.ay_limit/9.81:.3f} g, "
          f"grip_use={study.speed.grip_use:.2f}, süre={study.speed.t_end:.1f} s")
    bal = run_scenario(study, SCENARIOS[0])
    ok = validate_balanced(study, bal, os.path.join(OUT, "0_dogrulama_balanced.png"))
    write_csv(bal, os.path.join(OUT, "balanced.csv"))
    if stage == "balanced":
        print_events(study)
        return 0 if ok else 1
    if not ok:
        print("\nBalanced doğrulaması başarısız — diğer senaryolar çalıştırılmadı.")
        return 1
    from load_transfer_sim import visualize as V
    for sc in SCENARIOS[1:]:
        res = run_scenario(study, sc)
        write_csv(res, os.path.join(OUT, f"{sc.name}.csv"))
    print_events(study)
    write_events(study, os.path.join(OUT, "limit_olaylari.csv"))
    V.plot_traces(study, os.path.join(OUT, "1_arac_izi.png"))
    V.plot_grip(study, os.path.join(OUT, "2_grip_kullanimi.png"))
    V.plot_load_transfer(study, os.path.join(OUT, "3_yuk_transferi.png"))
    print("\nÇıktılar:", OUT)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else "all"))
