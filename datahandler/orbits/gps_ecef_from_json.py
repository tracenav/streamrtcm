import json
import math
import os
import sys
from dataclasses import dataclass
from datetime import datetime, timezone, timedelta
from typing import Any, Dict, List, Optional, Tuple
import numpy as np
from scipy.integrate import ode

# --- GLONASS RK4 propagator (use exact function as in orbits/glo.py) ---
J2 = 1.08262668e-3
EARTH_RADIUS_M = 6378136.0
MU_E_SI = 3.986004418e14  # m^3/s^2
OMEGA_E_SI = 7.2921151467e-5  # rad/s (more precise)

def _v_add(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> Tuple[float, float, float]:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])

def _v_scl(a: Tuple[float, float, float], s: float) -> Tuple[float, float, float]:
    return (a[0] * s, a[1] * s, a[2] * s)

def _accel_gravity_J2(r: Tuple[float, float, float]) -> Tuple[float, float, float]:
    x, y, z = r
    r2 = x * x + y * y + z * z
    rmag = math.sqrt(r2)
    if rmag == 0.0:
        return (0.0, 0.0, 0.0)
    r5 = r2 * r2 * rmag
    # Newtonian
    ax_N = -MU_E_SI * x / (rmag ** 3)
    ay_N = -MU_E_SI * y / (rmag ** 3)
    az_N = -MU_E_SI * z / (rmag ** 3)
    # J2
    cJ2 = -1.5 * J2 * MU_E_SI * (EARTH_RADIUS_M ** 2) / r5
    z2r2 = (z * z) / r2
    fxy = 1.0 - 5.0 * z2r2
    fz = 3.0 - 5.0 * z2r2
    ax_J2 = cJ2 * x * fxy
    ay_J2 = cJ2 * y * fxy
    az_J2 = cJ2 * z * fz
    return (ax_N + ax_J2, ay_N + ay_J2, az_N + az_J2)

def _accel_rotating_terms(r: Tuple[float, float, float], v: Tuple[float, float, float]) -> Tuple[float, float, float]:
    x, y, _ = r
    vx, vy, _ = v
    # Coriolis: -2Ω×v = (+2Ω vy, -2Ω vx, 0)
    a_cor = (2.0 * OMEGA_E_SI * vy, -2.0 * OMEGA_E_SI * vx, 0.0)
    # Centrifugal: -Ω×(Ω×r) = (+ω^2 x, +ω^2 y, 0)
    w2 = OMEGA_E_SI * OMEGA_E_SI
    a_cen = (w2 * x, w2 * y, 0.0)
    return _v_add(a_cor, a_cen)

def _total_accel(r: Tuple[float, float, float], v: Tuple[float, float, float], a_bcast: Tuple[float, float, float]) -> Tuple[float, float, float]:
    ag = _accel_gravity_J2(r)
    arf = _accel_rotating_terms(r, v)
    return (ag[0] + arf[0] + a_bcast[0], ag[1] + arf[1] + a_bcast[1], ag[2] + arf[2] + a_bcast[2])

def _rk4_step(r: Tuple[float, float, float], v: Tuple[float, float, float], dt: float,
              a_func) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    a1 = a_func(r, v)
    k1_r, k1_v = v, a1

    r2 = _v_add(r, _v_scl(k1_r, dt / 2.0))
    v2 = _v_add(v, _v_scl(k1_v, dt / 2.0))
    a2 = a_func(r2, v2)
    k2_r, k2_v = v2, a2

    r3 = _v_add(r, _v_scl(k2_r, dt / 2.0))
    v3 = _v_add(v, _v_scl(k2_v, dt / 2.0))
    a3 = a_func(r3, v3)
    k3_r, k3_v = v3, a3

    r4 = _v_add(r, _v_scl(k3_r, dt))
    v4 = _v_add(v, _v_scl(k3_v, dt))
    a4 = a_func(r4, v4)
    k4_r, k4_v = v4, a4

    r_next = (
        r[0] + (dt / 6.0) * (k1_r[0] + 2 * k2_r[0] + 2 * k3_r[0] + k4_r[0]),
        r[1] + (dt / 6.0) * (k1_r[1] + 2 * k2_r[1] + 2 * k3_r[1] + k4_r[1]),
        r[2] + (dt / 6.0) * (k1_r[2] + 2 * k2_r[2] + 2 * k3_r[2] + k4_r[2]),
    )
    v_next = (
        v[0] + (dt / 6.0) * (k1_v[0] + 2 * k2_v[0] + 2 * k3_v[0] + k4_v[0]),
        v[1] + (dt / 6.0) * (k1_v[1] + 2 * k2_v[1] + 2 * k3_v[1] + k4_v[1]),
        v[2] + (dt / 6.0) * (k1_v[2] + 2 * k2_v[2] + 2 * k3_v[2] + k4_v[2]),
    )
    return r_next, v_next

def _propagate_rk4_glonass(r0_m: Tuple[float, float, float], v0_m_s: Tuple[float, float, float],
                           a0_m_s2: Tuple[float, float, float], dt_total_s: float,
                           step_s: float = 1.0) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    n_full = int(abs(dt_total_s) // step_s)
    h = math.copysign(step_s, dt_total_s)
    r, v = r0_m, v0_m_s
    for _ in range(n_full):
        r, v = _rk4_step(r, v, h, lambda rr, vv: _total_accel(rr, vv, a0_m_s2))
    rem = dt_total_s - n_full * h
    if abs(rem) > 1e-9:
        r, v = _rk4_step(r, v, rem, lambda rr, vv: _total_accel(rr, vv, a0_m_s2))
    return r, v

# Exact dynamics/integrator akin to rtcm_ssr2osr.propagate_state (meters, m/s)
def _propagate_rtcm_dop853(r0_m: Tuple[float, float, float], v0_m_s: Tuple[float, float, float],
                           a0_m_s2: Tuple[float, float, float], dt_total_s: float,
                           step_s: float = 1.0) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    ax_ls, ay_ls, az_ls = a0_m_s2
    mu = MU_E_SI
    factor = -26332671177.69
    oe_2 = 5.3174941173225e-9
    oe2 = 1.4584230e-4

    def f(t, y):
        r = math.sqrt(y[0] * y[0] + y[1] * y[1] + y[2] * y[2])
        v_x, v_y, v_z = y[3], y[4], y[5]
        r2 = r * r
        r3 = r * r2
        r5 = r3 * r2
        gm3 = -mu / r3
        fr5 = factor / r5
        z2r = y[2] / r
        z2r = z2r * z2r
        z52r = 5.0 * z2r
        fxy = (gm3 + fr5 * (1.0 - z52r) + oe_2)
        a_x = fxy * y[0] + oe2 * v_y + ax_ls
        a_y = fxy * y[1] - oe2 * v_x + ay_ls
        a_z = (gm3 + fr5 * (3.0 - z52r)) * y[2] + az_ls
        return [v_x, v_y, v_z, a_x, a_y, a_z]

    # Integrate directly to final time; let dop853 manage internal steps
    t0 = 0.0
    tf = float(dt_total_s)
    y0 = np.array([r0_m[0], r0_m[1], r0_m[2], v0_m_s[0], v0_m_s[1], v0_m_s[2]], dtype=float)
    rint = ode(f).set_integrator('dop853')
    rint.set_initial_value(y0, t0)
    if tf == t0:
        y = y0
    else:
        rint.integrate(tf)
        y = rint.y
    return (float(y[0]), float(y[1]), float(y[2])), (float(y[3]), float(y[4]), float(y[5]))


GM = 3.986004418e14  # [m^3/s^2]
OMEGA_E = 7.2921151467e-5  # [rad/s]
GPS_EPOCH = datetime(1980, 1, 6, 0, 0, 0, tzinfo=timezone.utc)
BDS_EPOCH = datetime(2006, 1, 1, 0, 0, 0, tzinfo=timezone.utc)

# As of 2025-08-15 there are 18 leap seconds (GPS = UTC + 18 s)
LEAP_SECONDS_UTC_TO_GPS = 18
# Approximate GPS to BDT offset (BDT = GPS - 14 s)
GPS_TO_BDT_OFFSET = -14.0


@dataclass
class GPSEphemeris:
    prn: str
    week: int
    toe: float
    sqrt_a: float
    e: float
    m0: float
    dn: float
    omega0: float
    i0: float
    w: float
    omega_dot: float
    idot: float
    cuc: float
    cus: float
    crc: float
    crs: float
    cic: float
    cis: float
    toc: float
    af0: float
    af1: float
    af2: float
    tgd: float


@dataclass
class GaleEphemeris:
    prn: str
    week: int
    toe: float
    sqrt_a: float
    e: float
    m0: float
    dn: float
    omega0: float
    i0: float
    w: float
    omega_dot: float
    idot: float
    cuc: float
    cus: float
    crc: float
    crs: float
    cic: float
    cis: float
    toc: float
    af0: float
    af1: float
    af2: float


def parse_time_to_gps_week_sow(timestr: str, time_system: str = "UTC") -> Tuple[int, float]:
    s = timestr.strip().upper().replace("Z", "")
    # Accept both "YYYY-MM-DDTHH:MM:SS" and "YYYY-MM-DD HH:MM:SS"
    if "T" in s:
        dt_utc = datetime.fromisoformat(s).replace(tzinfo=timezone.utc)
    else:
        dt_utc = datetime.strptime(s, "%Y-%m-%d %H:%M:%S").replace(tzinfo=timezone.utc)
    # Convert to GPS time depending on declared time system
    ts = time_system.strip().upper()
    if ts == "GPS":
        dt_gps = dt_utc
    else:  # default: UTC
        dt_gps = dt_utc + timedelta(seconds=LEAP_SECONDS_UTC_TO_GPS)
    delta = (dt_gps - GPS_EPOCH).total_seconds()
    week = int(delta // 604800)
    sow = float(delta - week * 604800)
    return week, sow


def wrap_time_diff_seconds(dt_sec: float) -> float:
    # Wrap difference to [-302400, +302400]
    half_week = 302400.0
    if dt_sec > half_week:
        dt_sec -= 604800.0
    elif dt_sec < -half_week:
        dt_sec += 604800.0
    return dt_sec


def extract_gps_ephemerides_for_prn(ephemerides_json: Dict[str, Any], prn: str) -> List[GPSEphemeris]:
    out: List[GPSEphemeris] = []
    messages = ephemerides_json.get("messages", {}).get("1019", [])
    for entry in messages:
        dec = entry.get("decoded", {})
        sat_id = dec.get("sat_id")
        if not sat_id:
            continue
        if sat_id.upper() != prn.upper():
            continue
        try:
            eph = GPSEphemeris(
                prn=sat_id,
                week=int(dec["week"]),
                toe=float(dec["toe"]),
                sqrt_a=float(dec["root_a"]),
                e=float(dec["ecc"]),
                m0=float(dec["m0"]),
                dn=float(dec["dn"]),
                omega0=float(dec["omega_0"]),
                i0=float(dec["i0"]),
                w=float(dec["omega"]),
                omega_dot=float(dec["omega_dot"]),
                idot=float(dec["idot"]),
                cuc=float(dec["cuc"]),
                cus=float(dec["cus"]),
                crc=float(dec["crc"]),
                crs=float(dec["crs"]),
                cic=float(dec["cic"]),
                cis=float(dec["cis"]),
                toc=float(dec["toc"]),
                af0=float(dec["af_zero"]),
                af1=float(dec["af_one"]),
                af2=float(dec["af_two"]),
                tgd=float(dec.get("tgd", 0.0)),
            )
            out.append(eph)
        except KeyError:
            continue
    return out


def extract_gal_ephemerides_for_prn(ephemerides_json: Dict[str, Any], prn: str) -> List[GaleEphemeris]:
    out: List[GaleEphemeris] = []
    for entry in ephemerides_json.get("messages", {}).get("1046", []):
        dec = entry.get("decoded", {})
        sat_id = dec.get("sat_id")
        if not sat_id or sat_id.upper() != prn.upper():
            continue
        try:
            out.append(GaleEphemeris(
                prn=sat_id,
                week=int(dec["week"]),
                toe=float(dec["toe"]),  # seconds
                sqrt_a=float(dec["root_a"]),
                e=float(dec["ecc"]),
                m0=float(dec["m0"]),
                dn=float(dec["dn"]),
                omega0=float(dec["omega_0"]),
                i0=float(dec["i0"]),
                w=float(dec["omega"]),
                omega_dot=float(dec["omega_dot"]),
                idot=float(dec["idot"]),
                cuc=float(dec["cuc"]),
                cus=float(dec["cus"]),
                crc=float(dec["crc"]),
                crs=float(dec["crs"]),
                cic=float(dec["cic"]),
                cis=float(dec["cis"]),
                toc=float(dec["toc"]),
                af0=float(dec["af_zero"]),
                af1=float(dec["af_one"]),
                af2=float(dec["af_two"]),
            ))
        except KeyError:
            continue
    return out


@dataclass
class BDSEphemeris:
    prn: str
    week: int
    toe: float
    sqrt_a: float
    e: float
    m0: float
    dn: float
    omega0: float
    i0: float
    w: float
    omega_dot: float
    idot: float
    cuc: float
    cus: float
    crc: float
    crs: float
    cic: float
    cis: float
    toc: float
    af0: float
    af1: float
    af2: float


def extract_bds_ephemerides_for_prn(ephemerides_json: Dict[str, Any], prn: str) -> List[BDSEphemeris]:
    out: List[BDSEphemeris] = []
    for entry in ephemerides_json.get("messages", {}).get("1042", []):
        dec = entry.get("decoded", {})
        sat_id = dec.get("sat_id")
        if not sat_id or sat_id.upper() != prn.upper():
            continue
        try:
            out.append(BDSEphemeris(
                prn=sat_id,
                week=int(dec["week"]),
                toe=float(dec["toe"]),  # seconds (multiple of 8)
                sqrt_a=float(dec["root_a"]),
                e=float(dec["ecc"]),
                m0=float(dec["m0"]),
                dn=float(dec["dn"]),
                omega0=float(dec["omega_0"]),
                i0=float(dec["i0"]),
                w=float(dec["omega"]),
                omega_dot=float(dec["omega_dot"]),
                idot=float(dec["idot"]),
                cuc=float(dec["cuc"]),
                cus=float(dec["cus"]),
                crc=float(dec["crc"]),
                crs=float(dec["crs"]),
                cic=float(dec["cic"]),
                cis=float(dec["cis"]),
                toc=float(dec["toc"]),
                af0=float(dec["af_zero"]),
                af1=float(dec["af_one"]),
                af2=float(dec["af_two"]),
            ))
        except KeyError:
            continue
    return out


@dataclass
class GloState:
    prn: str
    tk: float  # [s] time mark within day (from message)
    x: float  # [m]
    y: float
    z: float
    vx: float  # [m/s]
    vy: float
    vz: float
    ax: float  # [m/s^2]
    ay: float
    az: float
    tau: float = 0.0    # [s] GLONASS clock bias at tb
    gamma: float = 0.0  # [s/s] relative frequency bias
    tb: float = 0.0     # [s] reference time tb
    nt: int = 0         # GLONASS day number in 4-year interval
    n4: int = 0         # GLONASS four-year interval number


def extract_glo_states_for_prn(ephemerides_json: Dict[str, Any], prn: str) -> List[GloState]:
    out: List[GloState] = []
    for entry in ephemerides_json.get("messages", {}).get("1020", []):
        dec = entry.get("decoded", {})
        sat_id = dec.get("sat_id")
        if not sat_id or sat_id.upper() != prn.upper():
            continue
        try:
            # positions in km, velocities in km/s, accelerations in km/s^2
            out.append(GloState(
                prn=sat_id,
                tk=float(dec.get("tk", dec.get("tb", 0.0))),
                x=float(dec["xn"]) * 1000.0,
                y=float(dec["yn"]) * 1000.0,
                z=float(dec["zn"]) * 1000.0,
                vx=float(dec["dxn"]) * 1000.0,
                vy=float(dec["dyn"]) * 1000.0,
                vz=float(dec["dzn"]) * 1000.0,
                ax=float(dec["ddxn"]) * 1000.0,
                ay=float(dec["ddyn"]) * 1000.0,
                az=float(dec["ddzn"]) * 1000.0,
                tau=float(dec.get("tau", 0.0)),
                gamma=float(dec.get("gamma", 0.0)),
                tb=float(dec.get("tb", 0.0)),
                nt=int(dec.get("nt", 0)),
                n4=int(dec.get("n4", 0)),
            ))
        except KeyError:
            continue
    return out


def choose_best_ephemeris(ephemerides: List[GPSEphemeris], t_week: int, t_sow: float) -> Optional[GPSEphemeris]:
    if not ephemerides:
        return None
    t_gps_abs = t_week * 604800.0 + t_sow
    best: Optional[Tuple[float, GPSEphemeris]] = None
    for e in ephemerides:
        toe_abs = e.week * 604800.0 + e.toe
        dt = t_gps_abs - toe_abs
        dt_wrapped = wrap_time_diff_seconds(dt)
        score = abs(dt_wrapped)
        if (best is None) or (score < best[0]):
            best = (score, e)
    return best[1] if best else None


def solve_kepler(M: float, e: float, tol: float = 1e-12, it_max: int = 50) -> float:
    # Newton-Raphson for E - e*sin(E) = M
    E = M
    for _ in range(it_max):
        f = E - e * math.sin(E) - M
        fp = 1.0 - e * math.cos(E)
        dE = -f / fp
        E += dE
        if abs(dE) < tol:
            break
    return E


def compute_ecef(eph: GPSEphemeris, t_week: int, t_sow: float) -> Tuple[float, float, float]:
    A = eph.sqrt_a * eph.sqrt_a
    n0 = math.sqrt(GM / (A ** 3))
    n = n0 + eph.dn

    t_abs = t_week * 604800.0 + t_sow
    toe_abs = eph.week * 604800.0 + eph.toe
    tk = wrap_time_diff_seconds(t_abs - toe_abs)

    M = eph.m0 + n * tk

    E = solve_kepler(M, eph.e)

    sinE = math.sin(E)
    cosE = math.cos(E)
    # True anomaly
    v = math.atan2(math.sqrt(1.0 - eph.e * eph.e) * sinE, cosE - eph.e)

    # Argument of latitude
    phi = v + eph.w

    # Second harmonic perturbations
    du = eph.cus * math.sin(2.0 * phi) + eph.cuc * math.cos(2.0 * phi)
    dr = eph.crs * math.sin(2.0 * phi) + eph.crc * math.cos(2.0 * phi)
    di = eph.cis * math.sin(2.0 * phi) + eph.cic * math.cos(2.0 * phi)

    u = phi + du
    r = A * (1.0 - eph.e * cosE) + dr
    i = eph.i0 + eph.idot * tk + di

    # Corrected longitude of ascending node
    Omega = eph.omega0 + (eph.omega_dot - OMEGA_E) * tk - OMEGA_E * eph.toe

    x_orb = r * math.cos(u)
    y_orb = r * math.sin(u)

    cosO = math.cos(Omega)
    sinO = math.sin(Omega)
    cosi = math.cos(i)
    sini = math.sin(i)

    x = x_orb * cosO - y_orb * cosi * sinO
    y = x_orb * sinO + y_orb * cosi * cosO
    z = y_orb * sini
    return x, y, z


def compute_clock_correction_seconds(eph: GPSEphemeris, t_week: int, t_sow: float) -> float:
    """
    Broadcast clock correction (seconds), excluding group delay TGD (for SP3 comparison).
    Δt_sv = af0 + af1*(t - toc) + af2*(t - toc)^2 + Δt_rel
    where Δt_rel = F * e * sqrt(a) * sin(E), F = -4.442807633e-10
    """
    # Time offsets
    t_abs = t_week * 604800.0 + t_sow
    toc_abs = eph.week * 604800.0 + eph.toc
    dt = wrap_time_diff_seconds(t_abs - toc_abs)

    # Mean motion
    A = eph.sqrt_a * eph.sqrt_a
    n0 = math.sqrt(GM / (A ** 3))
    n = n0 + eph.dn
    # Time from toe for orbital elements
    toe_abs = eph.week * 604800.0 + eph.toe
    tk = wrap_time_diff_seconds(t_abs - toe_abs)
    M = eph.m0 + n * tk
    E = solve_kepler(M, eph.e)

    # Relativistic correction
    F = -4.442807633e-10
    dtrel = F * eph.e * eph.sqrt_a * math.sin(E)

    # Broadcast clock model
    dt_sv = eph.af0 + eph.af1 * dt + eph.af2 * dt * dt + dtrel
    return dt_sv


def compute_ecef_gal(eph: GaleEphemeris, t_sow_gps: float) -> Tuple[float, float, float]:
    # Assume GPS~GST alignment; use SOW difference only
    A = eph.sqrt_a * eph.sqrt_a
    n0 = math.sqrt(GM / (A ** 3))
    n = n0 + eph.dn
    tk = wrap_time_diff_seconds(t_sow_gps - eph.toe)
    M = eph.m0 + n * tk
    E = solve_kepler(M, eph.e)
    sinE = math.sin(E)
    cosE = math.cos(E)
    v = math.atan2(math.sqrt(1.0 - eph.e * eph.e) * sinE, cosE - eph.e)
    phi = v + eph.w
    du = eph.cus * math.sin(2.0 * phi) + eph.cuc * math.cos(2.0 * phi)
    dr = eph.crs * math.sin(2.0 * phi) + eph.crc * math.cos(2.0 * phi)
    di = eph.cis * math.sin(2.0 * phi) + eph.cic * math.cos(2.0 * phi)
    u = phi + du
    r = A * (1.0 - eph.e * cosE) + dr
    i = eph.i0 + eph.idot * tk + di
    Omega = eph.omega0 + (eph.omega_dot - OMEGA_E) * tk - OMEGA_E * eph.toe
    x_orb = r * math.cos(u)
    y_orb = r * math.sin(u)
    cosO = math.cos(Omega)
    sinO = math.sin(Omega)
    cosi = math.cos(i)
    sini = math.sin(i)
    x = x_orb * cosO - y_orb * cosi * sinO
    y = x_orb * sinO + y_orb * cosi * cosO
    z = y_orb * sini
    return x, y, z


def compute_ecef_bds(eph: BDSEphemeris, t_sow_gps: float) -> Tuple[float, float, float]:
    # Convert GPS SOW to BDT SOW with fixed offset
    t_sow_bdt = (t_sow_gps + GPS_TO_BDT_OFFSET) % 604800.0
    A = eph.sqrt_a * eph.sqrt_a
    n0 = math.sqrt(GM / (A ** 3))
    n = n0 + eph.dn
    tk = wrap_time_diff_seconds(t_sow_bdt - eph.toe)
    M = eph.m0 + n * tk
    E = solve_kepler(M, eph.e)
    sinE = math.sin(E)
    cosE = math.cos(E)
    v = math.atan2(math.sqrt(1.0 - eph.e * eph.e) * sinE, cosE - eph.e)
    phi = v + eph.w
    du = eph.cus * math.sin(2.0 * phi) + eph.cuc * math.cos(2.0 * phi)
    dr = eph.crs * math.sin(2.0 * phi) + eph.crc * math.cos(2.0 * phi)
    di = eph.cis * math.sin(2.0 * phi) + eph.cic * math.cos(2.0 * phi)
    u = phi + du
    r = A * (1.0 - eph.e * cosE) + dr
    i = eph.i0 + eph.idot * tk + di
    Omega = eph.omega0 + (eph.omega_dot - OMEGA_E) * tk - OMEGA_E * eph.toe
    x_orb = r * math.cos(u)
    y_orb = r * math.sin(u)
    cosO = math.cos(Omega)
    sinO = math.sin(Omega)
    cosi = math.cos(i)
    sini = math.sin(i)
    x = x_orb * cosO - y_orb * cosi * sinO
    y = x_orb * sinO + y_orb * cosi * cosO
    z = y_orb * sini
    return x, y, z


def compute_ecef_glo(state: GloState, t_week: int, t_sow_gps: float) -> Tuple[float, float, float]:
    # Epochs: convert GPS to UTC, then shift +3h (Moscow) for GLONASS day/epoch handling
    gps_seconds = t_week * 604800.0 + t_sow_gps
    dt_gps = GPS_EPOCH + timedelta(seconds=gps_seconds)
    dt_utc = dt_gps - timedelta(seconds=LEAP_SECONDS_UTC_TO_GPS)
    dt_msc = dt_utc + timedelta(seconds=10800.0)
    # Seconds of day in Moscow time
    sod_msc = (dt_msc - dt_msc.replace(hour=0, minute=0, second=0, microsecond=0)).total_seconds()
    # Build t0 (broadcast epoch) in Moscow day from nt if available; otherwise use tb in UTC day as before
    # Here we follow tb within day, selecting previous or same tb relative to Moscow-day SOD
    if state.tb <= sod_msc:
        t0_msc = dt_msc.replace(hour=0, minute=0, second=0, microsecond=0) + timedelta(seconds=state.tb)
    else:
        t0_msc = (dt_msc.replace(hour=0, minute=0, second=0, microsecond=0) - timedelta(days=1)) + timedelta(seconds=state.tb)
    # Propagation delta in seconds (use Moscow timescale to mirror reference)
    dt_total = (dt_msc - t0_msc).total_seconds()

    # Initial state (SI)
    r0_m = (state.x, state.y, state.z)
    v0_m_s = (state.vx, state.vy, state.vz)
    a0_m_s2 = (state.ax, state.ay, state.az)

    # Propagate using exact RK4 integrator
    # Integrate using the RTCM dop853 dynamics for best match
    rT_m, _ = _propagate_rtcm_dop853(r0_m, v0_m_s, a0_m_s2, dt_total_s=dt_total, step_s=1.0)

    # Output in native PZ-90.11 frame (no rotation). If needed, apply precise 7-parameter transform externally.
    return float(rT_m[0]), float(rT_m[1]), float(rT_m[2])


def compute_clock_gps_microseconds(eph: GPSEphemeris, t_week: int, t_sow: float) -> float:
    return compute_clock_correction_seconds(eph, t_week, t_sow) * 1e6


def compute_clock_gal_microseconds(eph: GaleEphemeris, t_sow_gps: float) -> float:
    # Galileo broadcast clock similar model (ignoring SISA specifics here)
    dt = wrap_time_diff_seconds(t_sow_gps - eph.toc)
    # Relativistic correction term
    A = eph.sqrt_a * eph.sqrt_a
    n0 = math.sqrt(GM / (A ** 3))
    n = n0 + eph.dn
    M = eph.m0 + n * wrap_time_diff_seconds(t_sow_gps - eph.toe)
    E = solve_kepler(M, eph.e)
    F = -4.442807633e-10
    dtrel = F * eph.e * eph.sqrt_a * math.sin(E)
    clk = eph.af0 + eph.af1 * dt + eph.af2 * dt * dt + dtrel
    return clk * 1e6


def compute_clock_bds_microseconds(eph: BDSEphemeris, t_sow_gps: float) -> float:
    # Convert to BDT
    t_sow_bdt = (t_sow_gps + GPS_TO_BDT_OFFSET) % 604800.0
    dt = wrap_time_diff_seconds(t_sow_bdt - eph.toc)
    A = eph.sqrt_a * eph.sqrt_a
    n0 = math.sqrt(GM / (A ** 3))
    n = n0 + eph.dn
    M = eph.m0 + n * wrap_time_diff_seconds(t_sow_bdt - eph.toe)
    E = solve_kepler(M, eph.e)
    F = -4.442807633e-10
    dtrel = F * eph.e * eph.sqrt_a * math.sin(E)
    clk = eph.af0 + eph.af1 * dt + eph.af2 * dt * dt + dtrel
    return clk * 1e6


def compute_clock_glo_microseconds(st: GloState, t_sow_gps: float) -> float:
    # Use same Moscow-time normalization as propagation: dt = (MSC_target - t0_msc)
    dt_gps = GPS_EPOCH + timedelta(seconds=t_sow_gps)
    dt_utc = dt_gps - timedelta(seconds=LEAP_SECONDS_UTC_TO_GPS)
    dt_msc = dt_utc + timedelta(seconds=10800.0)
    sod_msc = (dt_msc - dt_msc.replace(hour=0, minute=0, second=0, microsecond=0)).total_seconds()
    if st.tb <= sod_msc:
        t0_msc = dt_msc.replace(hour=0, minute=0, second=0, microsecond=0) + timedelta(seconds=st.tb)
    else:
        t0_msc = (dt_msc.replace(hour=0, minute=0, second=0, microsecond=0) - timedelta(days=1)) + timedelta(seconds=st.tb)
    dt = (dt_msc - t0_msc).total_seconds()
    clk = st.tau + st.gamma * dt
    return clk * 1e6


def choose_best_glo_state(states: List[GloState], t_sow_gps: float, mode: str = "past") -> Optional[GloState]:
    if not states:
        return None
    t_utc_sow = (t_sow_gps - LEAP_SECONDS_UTC_TO_GPS) % 604800.0
    t_day = t_utc_sow % 86400.0
    mode = mode.lower()
    best = None
    best_score = None
    for st in states:
        dt = t_day - st.tb
        # compute raw forward and backward deltas (unwrapped)
        dt_forward = dt if dt <= 0 else dt - 86400.0  # negative or wrap to negative
        dt_backward = dt if dt >= 0 else dt + 86400.0  # positive or wrap to positive

        if mode == "future":
            # prefer dt <= 0 (tb >= t_day), take largest negative; else wrap to next day
            cand = dt if dt <= 0 else dt - 86400.0
            score = -cand  # larger negative is closer in future
        else:
            # past (default): take smallest positive delta from tb to t_day; if tb is after, wrap to previous day
            cand = dt if dt >= 0 else dt + 86400.0
            score = cand

        if best_score is None or score < best_score:
            best_score = score
            best = st
    return best


def main(argv: List[str]) -> None:
    if len(argv) < 2:
        print("Usage: python gps_ecef_from_json.py <ephemeris.json> [PRN|ALL|ALL-GNSS] [TIME_ISO] [TIME_SYS]")
        print("  TIME_SYS: UTC (default) or GPS")
        print("Examples:")
        print("  Single (UTC): python gps_ecef_from_json.py orbits/eph_20250815.json G01 2025-08-15T18:00:00Z UTC")
        print("  All GPS (GPS): python gps_ecef_from_json.py orbits/eph_20250815.json ALL 2025-08-15T18:00:00Z GPS")
        print("  All GNSS (GPS): python gps_ecef_from_json.py orbits/eph_20250815.json ALL-GNSS 2025-08-15T18:00:00Z GPS")
        sys.exit(1)

    json_path = argv[1]
    prn = argv[2] if len(argv) >= 3 else "G01"
    time_str = argv[3] if len(argv) >= 4 else "2025-08-15T18:00:00Z"
    time_sys = argv[4] if len(argv) >= 5 else "UTC"

    with open(json_path, "r", encoding="utf-8") as f:
        j = json.load(f)

    t_week, t_sow = parse_time_to_gps_week_sow(time_str, time_sys)
    # ALL mode: compute for all GPS PRNs present in 1019 messages
    mode = prn.strip().upper()
    if mode in ("ALL", "*"):
        # Collect distinct PRNs from 1019 entries
        prns: List[str] = []
        for entry in j.get("messages", {}).get("1019", []):
            sat_id = entry.get("decoded", {}).get("sat_id")
            if not sat_id:
                continue
            if sat_id.startswith("G") and sat_id not in prns:
                prns.append(sat_id)

        results: List[Dict[str, Any]] = []
        for p in sorted(prns):
            ephs = extract_gps_ephemerides_for_prn(j, p)
            if not ephs:
                continue
            eph = choose_best_ephemeris(ephs, t_week, t_sow)
            if eph is None:
                continue
            x, y, z = compute_ecef(eph, t_week, t_sow)
            clk_s = compute_clock_correction_seconds(eph, t_week, t_sow)
            results.append({
                "prn": p,
                "ecef_km": {"x": x/1000.0, "y": y/1000.0, "z": z/1000.0},
                "clock_microseconds": clk_s * 1e6
            })

        out = {
            "time": time_str,
            "time_system": time_sys.upper(),
            "gps_week": t_week,
            "gps_sow": t_sow,
            "results": results,
        }
        print(json.dumps(out, indent=2))
        return

    if mode == "ALL-GNSS":
        results: Dict[str, List[Dict[str, Any]]] = {"GPS": [], "GAL": [], "BDS": [], "GLO": []}

        # GPS
        gps_prns: List[str] = []
        for entry in j.get("messages", {}).get("1019", []):
            sid = entry.get("decoded", {}).get("sat_id")
            if sid and sid.startswith("G") and sid not in gps_prns:
                gps_prns.append(sid)
        for p in sorted(gps_prns):
            ephs = extract_gps_ephemerides_for_prn(j, p)
            eph = choose_best_ephemeris(ephs, t_week, t_sow) if ephs else None
            if not eph:
                continue
            x, y, z = compute_ecef(eph, t_week, t_sow)
            clk_us = compute_clock_gps_microseconds(eph, t_week, t_sow)
            results["GPS"].append({"prn": p, "ecef_km": {"x": x/1e3, "y": y/1e3, "z": z/1e3}, "clock_microseconds": clk_us})

        # GALILEO (E)
        gal_prns: List[str] = []
        for entry in j.get("messages", {}).get("1046", []):
            sid = entry.get("decoded", {}).get("sat_id")
            if sid and sid.startswith("E") and sid not in gal_prns:
                gal_prns.append(sid)
        for p in sorted(gal_prns):
            ephs = extract_gal_ephemerides_for_prn(j, p)
            eph = ephs[0] if ephs else None
            if not eph:
                continue
            x, y, z = compute_ecef_gal(eph, t_sow)
            clk_us = compute_clock_gal_microseconds(eph, t_sow)
            results["GAL"].append({"prn": p, "ecef_km": {"x": x/1e3, "y": y/1e3, "z": z/1e3}, "clock_microseconds": clk_us})

        # BDS (C)
        bds_prns: List[str] = []
        for entry in j.get("messages", {}).get("1042", []):
            sid = entry.get("decoded", {}).get("sat_id")
            if sid and sid.startswith("C") and sid not in bds_prns:
                bds_prns.append(sid)
        for p in sorted(bds_prns):
            ephs = extract_bds_ephemerides_for_prn(j, p)
            eph = ephs[0] if ephs else None
            if not eph:
                continue
            x, y, z = compute_ecef_bds(eph, t_sow)
            clk_us = compute_clock_bds_microseconds(eph, t_sow)
            results["BDS"].append({"prn": p, "ecef_km": {"x": x/1e3, "y": y/1e3, "z": z/1e3}, "clock_microseconds": clk_us})

        # GLONASS (R)
        glo_prns: List[str] = []
        for entry in j.get("messages", {}).get("1020", []):
            sid = entry.get("decoded", {}).get("sat_id")
            if sid and sid.startswith("R") and sid not in glo_prns:
                glo_prns.append(sid)
        for p in sorted(glo_prns):
            states = extract_glo_states_for_prn(j, p)
            st = choose_best_glo_state(states, t_sow, mode="past")
            if not st:
                continue
            x, y, z = compute_ecef_glo(st, t_week, t_sow)
            # GLONASS SP3 clock appears opposite sign relative to broadcast tau: invert sign to match SP3 convention
            clk_us = -compute_clock_glo_microseconds(st, t_sow)
            results["GLO"].append({"prn": p, "ecef_km": {"x": x/1e3, "y": y/1e3, "z": z/1e3}, "clock_microseconds": clk_us})

        out = {
            "time": time_str,
            "time_system": time_sys.upper(),
            "gps_week": t_week,
            "gps_sow": t_sow,
            "results": results,
        }
        print(json.dumps(out, indent=2))
        return

    # Single PRN mode
    ephemerides = extract_gps_ephemerides_for_prn(j, prn)
    if not ephemerides:
        raise RuntimeError(f"No 1019 ephemeris found for {prn} in {json_path}")
    eph = choose_best_ephemeris(ephemerides, t_week, t_sow)
    if eph is None:
        raise RuntimeError("Failed to select a suitable ephemeris record")
    x, y, z = compute_ecef(eph, t_week, t_sow)
    clk_s = compute_clock_correction_seconds(eph, t_week, t_sow)
    print(json.dumps({
        "prn": eph.prn,
        "time": time_str,
        "time_system": time_sys.upper(),
        "gps_week": t_week,
        "gps_sow": t_sow,
        "ecef_km": {"x": x/1000.0, "y": y/1000.0, "z": z/1000.0},
        "clock_microseconds": clk_s * 1e6
    }, indent=2))


def all_gnss_ecef_from_ephemeris(ephemerides: Dict[str, Any], time_str: str, time_sys: str = "UTC") -> Dict[str, Any]:
    """Compute ECEF and clocks for all constellations (ALL-GNSS) in-process.

    Returns the same structure previously printed to stdout in ALL-GNSS mode.
    """
    t_week, t_sow = parse_time_to_gps_week_sow(time_str, time_sys)
    results: Dict[str, List[Dict[str, Any]]] = {"GPS": [], "GAL": [], "BDS": [], "GLO": []}

    # GPS
    gps_prns: List[str] = []
    for entry in ephemerides.get("messages", {}).get("1019", []):
        sid = entry.get("decoded", {}).get("sat_id")
        if sid and sid.startswith("G") and sid not in gps_prns:
            gps_prns.append(sid)
    for p in sorted(gps_prns):
        ephs = extract_gps_ephemerides_for_prn(ephemerides, p)
        eph = choose_best_ephemeris(ephs, t_week, t_sow) if ephs else None
        if not eph:
            continue
        x, y, z = compute_ecef(eph, t_week, t_sow)
        clk_us = compute_clock_gps_microseconds(eph, t_week, t_sow)
        results["GPS"].append({"prn": p, "ecef_km": {"x": x/1e3, "y": y/1e3, "z": z/1e3}, "clock_microseconds": clk_us})

    # GALILEO
    gal_prns: List[str] = []
    for entry in ephemerides.get("messages", {}).get("1046", []):
        sid = entry.get("decoded", {}).get("sat_id")
        if sid and sid.startswith("E") and sid not in gal_prns:
            gal_prns.append(sid)
    for p in sorted(gal_prns):
        ephs = extract_gal_ephemerides_for_prn(ephemerides, p)
        eph = ephs[0] if ephs else None
        if not eph:
            continue
        x, y, z = compute_ecef_gal(eph, t_sow)
        clk_us = compute_clock_gal_microseconds(eph, t_sow)
        results["GAL"].append({"prn": p, "ecef_km": {"x": x/1e3, "y": y/1e3, "z": z/1e3}, "clock_microseconds": clk_us})

    # BDS
    bds_prns: List[str] = []
    for entry in ephemerides.get("messages", {}).get("1042", []):
        sid = entry.get("decoded", {}).get("sat_id")
        if sid and sid.startswith("C") and sid not in bds_prns:
            bds_prns.append(sid)
    for p in sorted(bds_prns):
        ephs = extract_bds_ephemerides_for_prn(ephemerides, p)
        eph = ephs[0] if ephs else None
        if not eph:
            continue
        x, y, z = compute_ecef_bds(eph, t_sow)
        clk_us = compute_clock_bds_microseconds(eph, t_sow)
        results["BDS"].append({"prn": p, "ecef_km": {"x": x/1e3, "y": y/1e3, "z": z/1e3}, "clock_microseconds": clk_us})

    # GLONASS
    glo_prns: List[str] = []
    for entry in ephemerides.get("messages", {}).get("1020", []):
        sid = entry.get("decoded", {}).get("sat_id")
        if sid and sid.startswith("R") and sid not in glo_prns:
            glo_prns.append(sid)
    for p in sorted(glo_prns):
        states = extract_glo_states_for_prn(ephemerides, p)
        st = choose_best_glo_state(states, t_sow, mode="past")
        if not st:
            continue
        x, y, z = compute_ecef_glo(st, t_week, t_sow)
        clk_us = -compute_clock_glo_microseconds(st, t_sow)
        results["GLO"].append({"prn": p, "ecef_km": {"x": x/1e3, "y": y/1e3, "z": z/1e3}, "clock_microseconds": clk_us})

    return {
        "time": time_str,
        "time_system": time_sys.upper(),
        "gps_week": t_week,
        "gps_sow": t_sow,
        "results": results,
    }


if __name__ == "__main__":
    main(sys.argv)


