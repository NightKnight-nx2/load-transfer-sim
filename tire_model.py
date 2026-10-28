"""
Pacejka "Magic Formula" lastik modeli.

Bu modül dogrudan hicbir ucuncu parti lastik kutuphanesini import etmez.
Kod, `commonroad-vehicle-models` (GPLv3) paketindeki tire_model.py
fonksiyonlarinin YAPISINI (long./lateral Pacejka katsayilari B,C,D,E ve
combined-slip friction-circle olceklemesi) referans alarak sifirdan
yazilmistir; sayisal katsayilar da kendi (basitlestirilmis, tipik bir
yolcu/spor otomobil lastigine ait) degerlerimizdir.

Bu dosyanın ek olarak sagladigi seyler:
  * tepe sürtünme / tepe kuvvet / tepe kayma açısı yardımcıları,
  * Fx talebi verildiğinde Fy'yi friction-ellipse ile küçülten
    `lateral_force_with_fx` (hız profili önceden verildiği için boylamsal
    kuvvet kayma oranından değil, talepten gelir).

Yük hassasiyeti: mu(Fz) = mu_nom * (1 - s*(Fz/Fz_nom - 1)), yani
D = mu(Fz)*Fz Fz'nin içbükey (concave) bir fonksiyonudur — fazla yüklenen
teker, azalan oranda ek tutunma kazanır. Bu, yük transferinin toplam aks
tutunmasını NEDEN azalttığının tek sebebidir.
"""
from __future__ import annotations

import numpy as np
from dataclasses import dataclass


@dataclass
class PacejkaParams:
    """Tek bir lastik icin sadelestirilmis Magic Formula katsayilari.

    Degerler, tipik bir performans lastigi icin makul mertebede secilmis
    "ilk-prensip" (sıfırdan) katsayilardir; belirli bir gercek lastigin
    olcum verisi degildir. mu_nominal, Fz_nominal yukunde D=mu*Fz iliskisini
    kalibre eder; D, yuk arttikca hafif "load sensitivity" (yuk hassasiyeti)
    ile artar ama orantili artmaz (gercek lastiklerde de boyledir).
    """

    # Boylamsal (longitudinal) egri
    B_x: float = 12.0
    C_x: float = 1.65
    E_x: float = -0.6
    mu_x: float = 1.15          # nominal yukte boylamsal tepe surtunme katsayisi

    # Yanal (lateral) egri
    B_y: float = 9.5
    C_y: float = 1.30
    E_y: float = -1.2
    mu_y: float = 1.05          # nominal yukte yanal tepe surtunme katsayisi

    Fz_nominal: float = 4000.0  # N, katsayilarin kalibre edildigi tekerlek yuku
    load_sensitivity: float = 0.20  # yuk arttikca mu'nun ne kadar dustugunu kontrol eder


def _peak_mu(mu_nom: float, Fz: float, Fz_nom: float, sensitivity: float) -> float:
    """Basit dogrusal yuk-hassasiyeti: mu(Fz) = mu_nom * (1 - sensitivity*(Fz/Fz_nom - 1))."""
    Fz = max(Fz, 1.0)
    ratio = Fz / Fz_nom
    mu = mu_nom * (1.0 - sensitivity * (ratio - 1.0))
    return max(mu, 0.05)


def pure_longitudinal_force(kappa: np.ndarray, Fz: np.ndarray, p: PacejkaParams) -> np.ndarray:
    """Saf boylamsal kayma (kappa, slip ratio, boyutsuz) icin Pacejka kuvveti [N]."""
    mu = _peak_mu(p.mu_x, Fz, p.Fz_nominal, p.load_sensitivity)
    D = mu * Fz
    Bx_kappa = p.B_x * kappa
    return D * np.sin(p.C_x * np.arctan(Bx_kappa - p.E_x * (Bx_kappa - np.arctan(Bx_kappa))))


def pure_lateral_force(alpha: np.ndarray, Fz: np.ndarray, p: PacejkaParams) -> np.ndarray:
    """Saf yanal kayma acisi (alpha, radyan) icin Pacejka kuvveti [N]."""
    mu = _peak_mu(p.mu_y, Fz, p.Fz_nominal, p.load_sensitivity)
    D = mu * Fz
    By_alpha = p.B_y * alpha
    return -D * np.sin(p.C_y * np.arctan(By_alpha - p.E_y * (By_alpha - np.arctan(By_alpha))))


def combined_slip_forces(kappa, alpha, Fz, p: PacejkaParams):
    """
    Birlesik (combined) kayma: once her eksende "saf" Pacejka kuvvetini
    hesapla, sonra ikisini birlikte bir surtunme cemberine (friction
    ellipse) sigacak sekilde olcekle.

    Girisler skaler veya ayni sekle sahip numpy dizileri olabilir.
    Donus: (Fx, Fy) [N]
    """
    Fx0 = pure_longitudinal_force(kappa, Fz, p)
    Fy0 = pure_lateral_force(alpha, Fz, p)

    mu_x = _peak_mu(p.mu_x, Fz, p.Fz_nominal, p.load_sensitivity)
    mu_y = _peak_mu(p.mu_y, Fz, p.Fz_nominal, p.load_sensitivity)

    # Friction-ellipse normalize edilmis kuvvet buyuklugu
    ex = Fx0 / np.maximum(mu_x * Fz, 1e-6)
    ey = Fy0 / np.maximum(mu_y * Fz, 1e-6)
    rho = np.sqrt(ex ** 2 + ey ** 2)
    scale = np.where(rho > 1.0, 1.0 / np.maximum(rho, 1e-9), 1.0)

    return Fx0 * scale, Fy0 * scale


def peak_mu_y(Fz: float, p: PacejkaParams) -> float:
    """Yanal tepe sürtünme katsayısı mu_y(Fz)."""
    return _peak_mu(p.mu_y, Fz, p.Fz_nominal, p.load_sensitivity)


def peak_mu_x(Fz: float, p: PacejkaParams) -> float:
    return _peak_mu(p.mu_x, Fz, p.Fz_nominal, p.load_sensitivity)


def peak_lateral_force(Fz: float, p: PacejkaParams) -> float:
    """Tek tekerin tepe yanal kuvveti D_y = mu_y(Fz)*Fz [N]."""
    Fz = max(Fz, 1.0)
    return peak_mu_y(Fz, p) * Fz


def peak_longitudinal_force(Fz: float, p: PacejkaParams) -> float:
    Fz = max(Fz, 1.0)
    return peak_mu_x(Fz, p) * Fz


def peak_slip_angle(p: PacejkaParams) -> float:
    """
    Saf yanal Fy(alpha) eğrisinin tepe yaptığı kayma açısı [rad].
    Tepe: C*atan(x) = pi/2, x = B*a - E*(B*a - atan(B*a)). x(a) monoton
    olduğundan bisection yeterli. Fz'den bağımsızdır (B,C,E sabit).
    """
    target = np.tan(np.pi / (2.0 * p.C_y))
    lo, hi = 0.0, 1.5
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        Ba = p.B_y * mid
        x = Ba - p.E_y * (Ba - np.arctan(Ba))
        if x < target:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def lateral_force_with_fx(alpha: float, Fz: float, Fx: float, p: PacejkaParams) -> float:
    """
    Saf Pacejka Fy'si, verilen boylamsal kuvvet Fx'i friction-ellipse içinde
    bırakacak şekilde ölçeklenir:  Fy = Fy0 * sqrt(1 - (Fx/(mu_x Fz))^2).
    Fx tepe boylamsal kuvveti aşarsa Fy = 0 (Fx ayrıca kırpılmaz; çağıran
    kontrol eder). İşaret kuralı: pozitif alpha -> negatif Fy.
    """
    Fz = max(Fz, 1.0)
    Fy0 = float(pure_lateral_force(alpha, Fz, p))
    fx = Fx / max(peak_longitudinal_force(Fz, p), 1e-6)
    s = np.sqrt(max(1.0 - fx * fx, 0.0))
    return Fy0 * s
