"""
Araç parametreleri (kütle, atalet, geometri, lastikler).

`VehicleParams`, load_transfer_sim'in referans aracını (kütle, Iz, dingil
mesafeleri, iz genişliği, CG yüksekliği, sürükleme, lastikler) tanımlar.
`bicycle_model.BicycleParams.from_sistem7()` buradan temel değerleri alır.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from load_transfer_sim.tire_model import PacejkaParams


@dataclass
class VehicleParams:
    mass: float = 1400.0          # kg
    Iz: float = 2100.0            # kg*m^2, yaw eylemsizlik momenti
    a: float = 1.25               # m, CG'den on aksa mesafe
    b: float = 1.45               # m, CG'den arka aksa mesafe
    track_width: float = 1.55     # m
    cg_height: float = 0.50       # m, kuazi-statik yuk transferi icin
    wheel_radius: float = 0.31    # m
    wheel_inertia: float = 1.6    # kg*m^2, tek tekerlek (jant+lastik) ataleti

    drag_coeff: float = 0.32
    frontal_area: float = 2.2     # m^2
    air_density: float = 1.225    # kg/m^3
    rolling_resistance: float = 0.014  # boyutsuz Crr

    max_engine_torque: float = 320.0   # N*m, tek arka teker basina, dusuk hizda
    max_brake_torque: float = 2200.0   # N*m, tek teker basina
    brake_bias_front: float = 0.6      # firen torkunun on aksa giden orani

    tire_front: PacejkaParams = field(default_factory=PacejkaParams)
    tire_rear: PacejkaParams = field(default_factory=PacejkaParams)

    g: float = 9.81
