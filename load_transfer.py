"""
Yük transferi: statik dağılım + boylamsal (pitch yerine) + yanal (roll).

İşaret kuralı (Sistem 7 ile aynı): x ileri, y SOL, ay > 0 = sola dönüş.
Sola dönüşte dış teker SAĞ'dır -> sağ tekerlerin yükü artar.
Teker sırası: FL, FR, RL, RR.

Yanal transfer modeli
---------------------
Toplam devirme momenti  M = m * ay * h_cg.  Bu moment, roll stiffness
oranına göre aksların arasında paylaşılır:

    phi_f = K_f / (K_f + K_r)                    (ön aksın payı)
    dFz_f = phi_f       * M / track_f            (ön aks, sol->sağ transfer [N])
    dFz_r = (1 - phi_f) * M / track_r

Moment dengesi dFz_f*track_f + dFz_r*track_r = M her zaman sağlanır; iki iz
genişliği eşitse dFz_f + dFz_r = m*ay*h/track ("toplam yük transferi").
Bu kuazi-statik, roll-center geometrisi içermeyen (yalnızca elastik) bir
modeldir: roll stiffness dağılımı dışında hiçbir şey yük transferini
aksların arasında kaydırmaz.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class RollStiffness:
    """Ön/arka roll stiffness [N*m/rad]. Yalnızca ORAN yük dağılımını belirler."""
    K_f: float = 55000.0
    K_r: float = 55000.0

    @property
    def front_share(self) -> float:
        return self.K_f / (self.K_f + self.K_r)

    @classmethod
    def from_share(cls, front_share: float, K_total: float = 110000.0) -> "RollStiffness":
        return cls(K_f=front_share * K_total, K_r=(1.0 - front_share) * K_total)


def axle_loads(mass: float, g: float, a: float, b: float, h_cg: float, ax: float):
    """Statik + boylamsal transfer. ax>0 (ivmelenme) arkaya yük kaydırır. -> (Fz_front, Fz_rear)."""
    L = a + b
    dlong = mass * ax * h_cg / L
    Fz_f = mass * g * b / L - dlong
    Fz_r = mass * g * a / L + dlong
    return max(Fz_f, 2.0), max(Fz_r, 2.0)


def lateral_transfer(mass: float, ay: float, h_cg: float,
                     track_f: float, track_r: float, roll: RollStiffness):
    """Aks başına sol->sağ yük transferi (dFz_f, dFz_r) [N]; ay>0 için pozitif."""
    M = mass * ay * h_cg
    phi = roll.front_share
    return phi * M / track_f, (1.0 - phi) * M / track_r


def wheel_loads(mass, g, a, b, h_cg, track_f, track_r, roll: RollStiffness, ax, ay):
    """
    Dört tekerin normal yükü [FL, FR, RL, RR] ve dökümü.
    Kaldırma (Fz<20 N) kırpılır; bu, yalnızca iç tekerin havalandığı uç
    durumda moment dengesini bozar (raporlanmaz, nadir).
    """
    Fz_f, Fz_r = axle_loads(mass, g, a, b, h_cg, ax)
    dF, dR = lateral_transfer(mass, ay, h_cg, track_f, track_r, roll)
    Fz = np.array([
        max(Fz_f / 2.0 - dF, 20.0), max(Fz_f / 2.0 + dF, 20.0),
        max(Fz_r / 2.0 - dR, 20.0), max(Fz_r / 2.0 + dR, 20.0),
    ])
    return Fz, dict(dFz_f=dF, dFz_r=dR, Fz_axle_f=Fz_f, Fz_axle_r=Fz_r)


def roll_angle(mass, g, ay, h_roll_arm, roll: RollStiffness) -> float:
    """Bilgi amaçlı gövde roll açısı [rad] (dinamiğe girmez). h_roll_arm: CG - roll ekseni mesafesi."""
    Kt = roll.K_f + roll.K_r
    return mass * ay * h_roll_arm / (Kt - mass * g * h_roll_arm)
