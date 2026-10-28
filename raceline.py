"""
Sabit raceline: basit bir "oval + S-viraj" kombinasyonu.

Yol, (uzunluk, egrilik) ciftlerinden olusan bir segment listesinden analitik
olarak (duz cizgi icin lineer, yay icin tam dairesel) insa edilir - sayisal
turev/eğri-uydurma gurultusu olmadan tam ve tutarli bir egrilik (curvature)
profili elde etmek icin. Kapali bir çember (lap) yerine ACIK bir yol
(baslangic != bitis) tercih edildi; boylece pure-pursuit kontrolorunde
"lap wraparound" karmasikligindan kacinilir.

Sıra: uzun duz -> genis U-donusu (oval'in bir ucu) -> orta duz ->
S-sikane (chicane) -> son duz -> genis U-donusu (oval'in diger ucu) ->
kisa duz. Boylece hem "oval" (genis yaricapli U donusleri) hem de
"S-viraj" (zit yonlu iki dar yay) tek bir yolda birlesir.
"""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class Raceline:
    s: np.ndarray        # kumulatif yay uzunlugu [m]
    x: np.ndarray        # dunya X [m]
    y: np.ndarray        # dunya Y [m]
    heading: np.ndarray  # yol tegetinin acisi [rad]
    curvature: np.ndarray  # imzali egrilik [1/m] (pozitif = sola donus)
    ds: float

    @property
    def length(self) -> float:
        return float(self.s[-1])

    def point_at_s(self, s_query: float):
        s_query = np.clip(s_query, self.s[0], self.s[-1])
        x = np.interp(s_query, self.s, self.x)
        y = np.interp(s_query, self.s, self.y)
        heading = np.interp(s_query, self.s, np.unwrap(self.heading))
        curvature = np.interp(s_query, self.s, self.curvature)
        return x, y, heading, curvature

    def nearest_s(self, x: float, y: float, s_guess: float, search_window: float = 25.0) -> float:
        """
        x,y'ye en yakin yol noktasinin s degerini bulur. Performans icin
        aramayi s_guess etrafinda kucuk bir pencereyle sinirlar (araç
        genelde yol uzerinde ilerledigi icin buna gerek yoktur, tam arama
        yapmak gereksiz yavas olur).
        """
        lo = max(self.s[0], s_guess - search_window)
        hi = min(self.s[-1], s_guess + search_window)
        mask = (self.s >= lo) & (self.s <= hi)
        if not np.any(mask):
            mask = slice(None)
        xs, ys, ss = self.x[mask], self.y[mask], self.s[mask]
        d2 = (xs - x) ** 2 + (ys - y) ** 2
        return float(ss[np.argmin(d2)])


def _arc_len(radius: float, angle_deg: float) -> float:
    return abs(radius) * np.radians(abs(angle_deg))


def build_oval_with_chicane(
    straight1=90.0,
    straight2=45.0,
    straight3=45.0,
    straight4=25.0,
    oval_radius=22.0,
    chicane_radius=16.0,
    chicane_angle_deg=28.0,
    ds=0.25,
) -> Raceline:
    """
    Segment listesi: (uzunluk_veya_None, egrilik). Yay segmentleri icin
    uzunluk _arc_len ile hesaplanir (egriligin tersinden ve verilen aciya
    gore); duz segmentler dogrudan metre cinsinden verilir.
    """
    segments = [
        ("straight", straight1, 0.0),
        ("arc", _arc_len(oval_radius, 180.0), 1.0 / oval_radius),      # oval'in 1. ucu (sola U-donusu)
        ("straight", straight2, 0.0),
        ("arc", _arc_len(chicane_radius, chicane_angle_deg), -1.0 / chicane_radius),   # S: sag
        ("arc", _arc_len(chicane_radius, 2 * chicane_angle_deg), 1.0 / chicane_radius),  # S: sol (2x aci)
        ("arc", _arc_len(chicane_radius, chicane_angle_deg), -1.0 / chicane_radius),   # S: sag (net heading degisimi 0)
        ("straight", straight3, 0.0),
        ("arc", _arc_len(oval_radius, 180.0), 1.0 / oval_radius),      # oval'in 2. ucu
        ("straight", straight4, 0.0),
    ]

    xs, ys, heads, kappas, esses = [0.0], [0.0], [0.0], [0.0], [0.0]
    x, y, heading, s = 0.0, 0.0, 0.0, 0.0

    for kind, length, kappa in segments:
        n_steps = max(1, int(np.ceil(length / ds)))
        step = length / n_steps
        for _ in range(n_steps):
            if abs(kappa) < 1e-9:
                x += step * np.cos(heading)
                y += step * np.sin(heading)
            else:
                dtheta = kappa * step
                R = 1.0 / kappa
                # Tam dairesel adim (Euler entegrasyonundan daha dogru):
                x += R * (np.sin(heading + dtheta) - np.sin(heading))
                y += -R * (np.cos(heading + dtheta) - np.cos(heading))
                heading += dtheta
            s += step
            xs.append(x); ys.append(y); heads.append(heading); kappas.append(kappa); esses.append(s)

    return Raceline(
        s=np.array(esses), x=np.array(xs), y=np.array(ys),
        heading=np.array(heads), curvature=np.array(kappas), ds=ds,
    )
