#!/usr/bin/env python3

'''Replay proposed precision-landing descent and miss-abort rules over Copter logs.

Stock landing logs already contain the radar estimate (PL: pX pY vX vY mZ),
the attitude (ATT) and, when fitted, the downward rangefinder (RFND) and
altitude (POS / CTUN). This script uses those to answer two questions the
logged trajectory can actually answer:

1. Open loop. At each radar sample in the final 0.35 m, what descent rate
   would the timed-descent rule have commanded, and would the miss-abort
   have fired? This is the false-alarm check against real approaches.

2. Short closed loop. From the logged horizontal state at 0.35 m, integrate
   a point mass that brakes the way AC_PosControl predicts a stop (sqrt
   controller, P gain, horizontal accel limit). Compare touchdown error if
   the last 0.35 m is flown at full LAND_SPD_MS against the timed-descent
   rate. The logged path itself is not re-flown: it was produced by today's
   controller.

What this is not: ArduPilot Tools/Replay is an EKF replay. It will not swap
the landing controller inside a BIN. A full sideways-crossing or wind case
still belongs in SITL (PLND_TYPE=4, SIM_PLD_*, SIM_WIND_*). Use the logs to
see the real error, speed and lean at 0.35 m, and to reject thresholds that
would have aborted good landings.

AP_FLAKE8_CLEAN
'''

import argparse
import csv
import math
import os
import sys

# Final-stage gate in Mode::land_run_vertical_control. The timed descent
# replaces the "full LAND_SPD_MS regardless of error" behaviour below this.
FINAL_HEIGHT_M = 0.35
# Stock slowdown band above the final stage.
SLOWDOWN_HEIGHT_M = 2.0
# "A few centimetres" capture radius.
CAPTURE_RADIUS_M = 0.03
# Descent is height/time, but never slower than this while capture is accepted,
# and never faster than LAND_SPD_MS.
MIN_DESCENT_MS = 0.05
# Hold rather than creep in if braking will not arrive inside this time.
MAX_CAPTURE_TIME_S = 5.0
# Outbound faster than this counts as moving away.
MOVING_AWAY_MS = 0.05
# Latch descent rate (and, when measurement noise is applied, the horizontal
# aim) once this close to the surface.
FREEZE_HEIGHT_M = 0.12
# Miss-abort window and thresholds from the pin-landing brief.
ABORT_WINDOW_M = 1.5
BELOW_PAD_M = 0.12
BELOW_PAD_FRAMES = 2
LEAN_ABORT_DEG = 10.0
LEAN_HOLD_DEG = 20.0
ATTITUDE_ERROR_ABORT_DEG = 10.0
LEAN_PERSIST_S = 0.15
# Remember the pad top only while the radar solution is this close horizontally.
PAD_MEMORY_ERR_M = 0.15
PAD_MEMORY_MIN_MZ_M = 0.20
# Stock precland slowdown constants in land_run_vertical_control.
STOCK_ACCEPT_ERR_M = 0.15
STOCK_MIN_DESCENT_MS = 0.1
# Copter mode number and land-complete event id.
MODE_LAND = 9
EV_LAND_COMPLETE = 18

# Defaults used when the log has no matching PARM. Copter 4.7 names.
DEFAULT_LAND_SPD_MS = 0.5
DEFAULT_POS_P = 1.0          # PSC_NE_POS_P
DEFAULT_ACCEL_MSS = 2.5      # WP_ACC
DEFAULT_VEL_MAX_MS = 10.0    # WP_SPD
DEFAULT_XY_DIST_MAX_M = 2.5  # PLND_XY_DIST_MAX

DT_S = 0.01
SIM_LIMIT_S = 20.0


def _clamp(value, low, high):
    return max(low, min(high, value))


def _hypot(x, y):
    return math.hypot(x, y)


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1]


def sqrt_controller(error, p, second_ord_lim):
    '''Scalar form of AP_Math::sqrt_controller without the dt clamp.'''
    if second_ord_lim <= 0.0:
        return error * p
    if p == 0.0:
        if error > 0.0:
            return math.sqrt(2.0 * second_ord_lim * error)
        if error < 0.0:
            return -math.sqrt(2.0 * second_ord_lim * (-error))
        return 0.0
    linear_dist = second_ord_lim / (p * p)
    if error > linear_dist:
        return math.sqrt(2.0 * second_ord_lim * (error - (linear_dist * 0.5)))
    if error < -linear_dist:
        return -math.sqrt(2.0 * second_ord_lim * ((-error) - (linear_dist * 0.5)))
    return error * p


def stopping_distance(velocity, p, accel_max):
    '''AP_Math::stopping_distance, the inverse of sqrt_controller.'''
    return _inv_sqrt_controller(velocity, p, accel_max)


def _inv_sqrt_controller(output, p, d_max):
    if d_max > 0.0 and p == 0.0:
        return (output * output) / (2.0 * d_max)
    if d_max <= 0.0 and p != 0.0:
        return output / p
    if d_max <= 0.0 and p == 0.0:
        return 0.0
    linear_velocity = d_max / p
    if abs(output) < linear_velocity:
        return output / p
    linear_dist = d_max / (p * p)
    stopping = (linear_dist * 0.5) + (output * output) / (2.0 * d_max)
    if output > 0.0:
        return stopping
    return -stopping


def _limit_delta(dx, dy, max_step):
    mag = _hypot(dx, dy)
    if mag <= max_step or mag <= 0.0:
        return dx, dy
    scale = max_step / mag
    return dx * scale, dy * scale


def step_horizontal(err, err_dot, dt, pos_p, accel, vel_max, aim=None):
    '''Advance target-relative error using accel-limited sqrt-controller braking.

    err is the vector from the vehicle to the aim point (the pin, unless the
    horizontal target has been frozen). Positive err_dot grows that error.
    The velocity demand is the copter position P gain passed through
    sqrt_controller, then slewed at accel (m/s^2). This is the same pair of
    limits AC_PosControl::get_stopping_point_NE_m uses.
    '''
    if aim is None:
        aim = (0.0, 0.0)
    # Error of the vehicle relative to the latched aim, in the same frame.
    to_aim = (aim[0] - err[0], aim[1] - err[1])
    dist = _hypot(to_aim[0], to_aim[1])
    if dist <= 1.0e-6:
        v_dem = (0.0, 0.0)
    else:
        speed = min(sqrt_controller(dist, pos_p, accel), vel_max)
        # err_dot is the derivative of vehicle-to-pin error. Moving err toward
        # the aim point (the pin, or a latched aim) is unit(aim - err) * speed.
        v_dem = (to_aim[0] / dist * speed, to_aim[1] / dist * speed)
    dvx, dvy = _limit_delta(v_dem[0] - err_dot[0], v_dem[1] - err_dot[1], accel * dt)
    err_dot = (err_dot[0] + dvx, err_dot[1] + dvy)
    err = (err[0] + err_dot[0] * dt, err[1] + err_dot[1] * dt)
    return err, err_dot


def _segment_miss_distance(e0, e1):
    '''Closest approach of the segment e0->e1 to the origin.'''
    dx = e1[0] - e0[0]
    dy = e1[1] - e0[1]
    denom = dx * dx + dy * dy
    if denom <= 0.0:
        return _hypot(e0[0], e0[1]), 0.0
    t = _clamp(-(e0[0] * dx + e0[1] * dy) / denom, 0.0, 1.0)
    x = e0[0] + t * dx
    y = e0[1] + t * dy
    return _hypot(x, y), t


def time_to_capture(err, err_dot, pos_p, accel, vel_max, capture_m=CAPTURE_RADIUS_M, limit_s=MAX_CAPTURE_TIME_S):
    '''Seconds for the braking model to enter capture_m. None if it does not.'''
    dist = _hypot(err[0], err[1])
    if dist <= capture_m:
        return 0.0
    e = err
    ed = err_dot
    t = 0.0
    while t < limit_s:
        e_next, ed_next = step_horizontal(e, ed, DT_S, pos_p, accel, vel_max)
        miss, frac = _segment_miss_distance(e, e_next)
        if miss <= capture_m:
            return t + frac * DT_S
        e, ed = e_next, ed_next
        t += DT_S
    return None


def closing_speed_ms(err, err_dot):
    '''Positive when the target-relative error is shrinking.'''
    dist = _hypot(err[0], err[1])
    if dist <= 1.0e-6:
        return 0.0
    return -_dot(err, err_dot) / dist


def timed_descent_ms(height_m, err, err_dot, land_spd_ms, pos_p, accel, vel_max):
    '''Descent rate (m/s, positive down) for the final stage.

    Returns 0 when the vehicle is moving away or capture is too slow.
    Returns None above FINAL_HEIGHT_M; the caller keeps the stock schedule.
    '''
    if height_m > FINAL_HEIGHT_M:
        return None
    dist = _hypot(err[0], err[1])
    if dist <= CAPTURE_RADIUS_M:
        return land_spd_ms
    if closing_speed_ms(err, err_dot) < -MOVING_AWAY_MS:
        return 0.0
    capture_s = time_to_capture(err, err_dot, pos_p, accel, vel_max)
    if capture_s is None or capture_s > MAX_CAPTURE_TIME_S:
        return 0.0
    if capture_s < 1.0e-3:
        return land_spd_ms
    return _clamp(height_m / capture_s, MIN_DESCENT_MS, land_spd_ms)


def stock_descent_ms(height_m, err_m, land_spd_ms, xy_dist_max_m):
    '''Descent rate today's land_run_vertical_control would request.'''
    if xy_dist_max_m > 0.0 and err_m > xy_dist_max_m:
        return 0.0
    if FINAL_HEIGHT_M < height_m < SLOWDOWN_HEIGHT_M:
        max_descent = abs(land_spd_ms) * 0.5
        slowdown = max(0.0, err_m * (max_descent / STOCK_ACCEPT_ERR_M))
        # Matches MIN(-min_descent, -max_descent + slowdown), as a positive rate.
        return max(STOCK_MIN_DESCENT_MS, max_descent - slowdown)
    return abs(land_spd_ms)


def simulate_touchdown(err, err_dot, height_m, land_spd_ms, pos_p, accel, vel_max,
                       xy_dist_max_m, use_timed_descent):
    '''Integrate to height 0. The aim point is the pin. Below FREEZE_HEIGHT_M the
    timed-descent rate is latched so the last few centimetres are not re-planned.
    '''
    e = err
    ed = err_dot
    h = height_m
    t = 0.0
    latched_rate = None
    held = False
    while h > 0.0 and t < SIM_LIMIT_S:
        dist = _hypot(e[0], e[1])
        if use_timed_descent and h <= FINAL_HEIGHT_M:
            if h <= FREEZE_HEIGHT_M and latched_rate is not None:
                rate = latched_rate
            else:
                rate = timed_descent_ms(h, e, ed, land_spd_ms, pos_p, accel, vel_max)
                if rate is None:
                    rate = stock_descent_ms(h, dist, land_spd_ms, xy_dist_max_m)
                if h <= FREEZE_HEIGHT_M:
                    latched_rate = rate
            if rate <= 0.0:
                held = True
        else:
            rate = stock_descent_ms(h, dist, land_spd_ms, xy_dist_max_m)

        e, ed = step_horizontal(e, ed, DT_S, pos_p, accel, vel_max)
        if rate > 0.0:
            h -= rate * DT_S
            held = False
        t += DT_S
        if use_timed_descent and held and t > MAX_CAPTURE_TIME_S and h > FINAL_HEIGHT_M * 0.5:
            # Still refusing to descend after the capture budget. Report where
            # the vehicle is rather than burning the rest of the time limit.
            break
    return {
        'err_m': _hypot(e[0], e[1]),
        'height_m': max(0.0, h),
        'time_s': t,
        'touched_down': h <= 0.0,
    }


def _median(values):
    if not values:
        return None
    ordered = sorted(values)
    mid = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[mid]
    return 0.5 * (ordered[mid - 1] + ordered[mid])


def _as_float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _as_str(value):
    if isinstance(value, bytes):
        value = value.split(b'\x00', 1)[0].decode('utf-8', 'replace')
    return str(value).strip('\x00').strip()


class Params(object):
    '''Controller limits, filled from PARM when the log has them.'''

    def __init__(self):
        self.land_spd_ms = DEFAULT_LAND_SPD_MS
        self.pos_p = DEFAULT_POS_P
        self.accel_mss = DEFAULT_ACCEL_MSS
        self.vel_max_ms = DEFAULT_VEL_MAX_MS
        self.xy_dist_max_m = DEFAULT_XY_DIST_MAX_M

    def update(self, name, value):
        name = _as_str(name)
        value = _as_float(value)
        if value is None:
            return
        # Current names, then the pre-metres names still seen in older logs.
        if name in ('LAND_SPD_MS',):
            self.land_spd_ms = abs(value)
        elif name in ('LAND_SPEED',):
            self.land_spd_ms = abs(value) * 0.01
        elif name in ('PSC_NE_POS_P', 'PSC_POSXY_P'):
            if value > 0.0:
                self.pos_p = value
        elif name in ('WP_ACC',):
            if value > 0.0:
                self.accel_mss = value
        elif name in ('WPNAV_ACCEL',):
            if value > 0.0:
                self.accel_mss = value * 0.01
        elif name in ('WP_SPD',):
            if value > 0.0:
                self.vel_max_ms = value
        elif name in ('WPNAV_SPEED',):
            if value > 0.0:
                self.vel_max_ms = value * 0.01
        elif name in ('PLND_XY_DIST_MAX',):
            self.xy_dist_max_m = value


def _nearest(series, t_us):
    '''Last sample with t_us <= t, from a time-ordered list of (t_us, payload).'''
    if not series:
        return None
    lo = 0
    hi = len(series)
    while lo < hi:
        mid = (lo + hi) // 2
        if series[mid][0] <= t_us:
            lo = mid + 1
        else:
            hi = mid
    if lo == 0:
        return series[0][1]
    return series[lo - 1][1]


def analyse_landing(pl_samples, att, alt, rfnd, ctun, params, land_complete_us=None):
    '''Score one landing. pl_samples are dicts with t_us, px, py, vx, vy, mx, my, mz, tacq.'''
    acquired = [s for s in pl_samples if s['tacq'] and s['mz'] is not None and s['mz'] > 0.0]
    notes = []
    if not acquired:
        return {'notes': 'no acquired PL samples', 'usable': False}

    if land_complete_us is not None:
        in_air = [s for s in acquired if s['t_us'] <= land_complete_us]
        if in_air:
            acquired = in_air
        else:
            notes.append('land-complete before any PL sample; using the whole segment')

    min_sample = min(acquired, key=lambda s: s['mz'])
    logged_err = _hypot(min_sample['px'], min_sample['py'])

    # Pad-top altitude from radar while horizontally close. Altitude is only
    # used as the coordinate the radar range is subtracted from.
    pad_samples = []
    for s in acquired:
        err_m = _hypot(s['px'], s['py'])
        if err_m > PAD_MEMORY_ERR_M or s['mz'] < PAD_MEMORY_MIN_MZ_M:
            continue
        alt_m = _nearest(alt, s['t_us'])
        if alt_m is None:
            continue
        pad_samples.append(alt_m - s['mz'])
    pad_top = _median(pad_samples)
    # Closest radar range while the solution was tight. A later range that
    # stays longer than this, with the solution no longer tight, is the beam
    # leaving the pin for the ground. One step is not enough; it has to hold.
    centered_mz = [s['mz'] for s in acquired
                   if _hypot(s['px'], s['py']) <= PAD_MEMORY_ERR_M and s['mz'] >= PAD_MEMORY_MIN_MZ_M]
    pin_mz = min(centered_mz) if centered_mz else None

    below_pad = False
    below_run = 0
    below_reason = ''
    mz_off_pin = 0
    descent_cmds = []
    stock_cmds = []
    actual_desired = []

    for s in acquired:
        h = s['mz']
        err = (s['px'], s['py'])
        err_dot = (s['vx'], s['vy'])
        err_m = _hypot(s['px'], s['py'])
        alt_m = _nearest(alt, s['t_us'])
        above_pad = None
        if pad_top is not None and alt_m is not None:
            above_pad = alt_m - pad_top
        in_window = h <= ABORT_WINDOW_M or (above_pad is not None and above_pad <= ABORT_WINDOW_M)

        if in_window and above_pad is not None and above_pad < -BELOW_PAD_M:
            below_run += 1
            if below_run >= BELOW_PAD_FRAMES:
                below_pad = True
                below_reason = 'altitude %.2f m below radar pad top' % (-above_pad)
        else:
            below_run = 0

        off_pin = (in_window and pin_mz is not None and err_m > PAD_MEMORY_ERR_M
                   and s['mz'] > pin_mz + BELOW_PAD_M)
        if off_pin:
            mz_off_pin += 1
            if mz_off_pin >= BELOW_PAD_FRAMES:
                below_pad = True
                below_reason = 'radar range left the pin for the ground'
        else:
            mz_off_pin = 0

        if h <= FINAL_HEIGHT_M:
            cmd = timed_descent_ms(h, err, err_dot, params.land_spd_ms, params.pos_p, params.accel_mss, params.vel_max_ms)
            if h <= FREEZE_HEIGHT_M and descent_cmds:
                cmd = descent_cmds[-1]
            descent_cmds.append(cmd)
            stock_cmds.append(stock_descent_ms(h, err_m, params.land_spd_ms, params.xy_dist_max_m))
            c = _nearest(ctun, s['t_us'])
            if c is not None:
                # CTUN.DCRt is climb rate, positive up.
                actual_desired.append(-c)

    # Attitude is logged faster than the radar. Walk ATT so a 150 ms lean is
    # not missed between PL samples, and ignore anything after land-complete.
    pl_series = [(s['t_us'], s) for s in acquired]
    lean_abort = False
    lean_hold = False
    lean_peak = 0.0
    att_err_peak = 0.0
    lean_run_us = None
    for t_us, (roll, pitch, des_roll, des_pitch) in att:
        if land_complete_us is not None and t_us > land_complete_us:
            break
        pl = _nearest(pl_series, t_us)
        if pl is None:
            continue
        alt_m = _nearest(alt, t_us)
        above_pad = None if pad_top is None or alt_m is None else alt_m - pad_top
        in_window = pl['mz'] <= ABORT_WINDOW_M or (above_pad is not None and above_pad <= ABORT_WINDOW_M)
        if not in_window:
            lean_run_us = None
            continue
        lean = max(abs(roll), abs(pitch))
        att_err = max(abs(roll - des_roll), abs(pitch - des_pitch))
        lean_peak = max(lean_peak, lean)
        att_err_peak = max(att_err_peak, att_err)
        hot = lean > LEAN_ABORT_DEG or att_err > ATTITUDE_ERROR_ABORT_DEG
        if not hot:
            lean_run_us = None
            continue
        if lean_run_us is None:
            lean_run_us = t_us
        elif (t_us - lean_run_us) >= int(LEAN_PERSIST_S * 1.0e6):
            lean_abort = True
            if lean > LEAN_HOLD_DEG:
                lean_hold = True

    # Downward rangefinder leaving the pin for the ground. Remember the range
    # while the radar solution is tight, then require a longer range twice.
    if rfnd:
        pin_range = []
        for t_us, dist in rfnd:
            if land_complete_us is not None and t_us > land_complete_us:
                break
            pl = _nearest(pl_series, t_us)
            if pl is None or _hypot(pl['px'], pl['py']) > PAD_MEMORY_ERR_M:
                continue
            if PAD_MEMORY_MIN_MZ_M <= pl['mz'] <= ABORT_WINDOW_M:
                pin_range.append(dist)
        pin_rfnd = _median(pin_range)
        if pin_rfnd is not None:
            rfnd_hit = 0
            for t_us, dist in rfnd:
                if land_complete_us is not None and t_us > land_complete_us:
                    break
                pl = _nearest(pl_series, t_us)
                if pl is None or pl['mz'] > ABORT_WINDOW_M:
                    rfnd_hit = 0
                    continue
                err_m = _hypot(pl['px'], pl['py'])
                if err_m > PAD_MEMORY_ERR_M and dist > pin_rfnd + BELOW_PAD_M:
                    rfnd_hit += 1
                    if rfnd_hit >= BELOW_PAD_FRAMES:
                        below_pad = True
                        below_reason = 'rangefinder left the pin for the ground'
                else:
                    rfnd_hit = 0

    crossing = None
    for s in acquired:
        if s['mz'] <= FINAL_HEIGHT_M:
            crossing = s
            break

    sim_new = None
    sim_stock = None
    if crossing is not None:
        err = (crossing['px'], crossing['py'])
        err_dot = (crossing['vx'], crossing['vy'])
        sim_stock = simulate_touchdown(
            err, err_dot, crossing['mz'], params.land_spd_ms, params.pos_p,
            params.accel_mss, params.vel_max_ms, params.xy_dist_max_m, False)
        sim_new = simulate_touchdown(
            err, err_dot, crossing['mz'], params.land_spd_ms, params.pos_p,
            params.accel_mss, params.vel_max_ms, params.xy_dist_max_m, True)

    held_frac = None
    if descent_cmds:
        held_frac = sum(1 for c in descent_cmds if c <= 0.0) / float(len(descent_cmds))

    # mX/mY is the raw measurement, pX/pY the lag-compensated output. The
    # difference includes camera and landing offsets as well as radar noise.
    residuals = []
    for s in acquired:
        if s['mx'] is None or s['my'] is None:
            continue
        residuals.append(_hypot(s['mx'] - s['px'], s['my'] - s['py']))
    residual_rms = None
    if residuals:
        residual_rms = math.sqrt(sum(v * v for v in residuals) / float(len(residuals)))

    result = {
        'usable': True,
        'n_pl': len(acquired),
        'logged_err_m': logged_err,
        'logged_min_mz_m': min_sample['mz'],
        'pad_top_known': pad_top is not None,
        'pad_top_m': pad_top,
        'below_pad': below_pad,
        'below_reason': below_reason,
        'lean_abort': lean_abort,
        'lean_hold': lean_hold,
        'lean_peak_deg': lean_peak,
        'att_err_peak_deg': att_err_peak,
        'timed_descent_median_ms': _median(descent_cmds),
        'stock_descent_median_ms': _median(stock_cmds),
        'actual_desired_median_ms': _median(actual_desired),
        'held_fraction': held_frac,
        'final_samples': len(descent_cmds),
        'sim_new_err_m': None if sim_new is None else sim_new['err_m'],
        'sim_stock_err_m': None if sim_stock is None else sim_stock['err_m'],
        'sim_new_touched': None if sim_new is None else sim_new['touched_down'],
        'sim_stock_touched': None if sim_stock is None else sim_stock['touched_down'],
        'crossing_mz_m': None if crossing is None else crossing['mz'],
        'crossing_err_m': None if crossing is None else _hypot(crossing['px'], crossing['py']),
        'crossing_speed_ms': None if crossing is None else _hypot(crossing['vx'], crossing['vy']),
        'meas_residual_rms_m': residual_rms,
        'land_spd_ms': params.land_spd_ms,
        'pos_p': params.pos_p,
        'accel_mss': params.accel_mss,
        'notes': '; '.join(notes),
    }
    return result


def format_landing(path, index, result):
    if not result.get('usable'):
        return '%s landing %d: %s' % (path, index, result.get('notes', 'unusable'))
    lines = [
        '%s landing %d: %d PL samples, LAND_SPD_MS %.2f, PSC_NE_POS_P %.2f, WP_ACC %.2f' % (
            path, index, result['n_pl'], result['land_spd_ms'], result['pos_p'], result['accel_mss']),
        '  logged estimator error at minimum mZ: %.3f m (mZ %.3f m)' % (
            result['logged_err_m'], result['logged_min_mz_m']),
    ]
    if result.get('meas_residual_rms_m') is not None:
        lines.append(
            '  radar measurement vs estimator rms: %.3f m (includes camera offset and lag)' % (
                result['meas_residual_rms_m']))
    if result['final_samples']:
        lines.append(
            '  open-loop below %.2f m: timed descent median %.2f m/s, stock median %.2f m/s, '
            'logged desired median %s, held on %.0f%% of samples' % (
                FINAL_HEIGHT_M,
                result['timed_descent_median_ms'],
                result['stock_descent_median_ms'],
                'n/a' if result['actual_desired_median_ms'] is None else '%.2f m/s' % result['actual_desired_median_ms'],
                100.0 * (result['held_fraction'] or 0.0)))
    else:
        lines.append('  open-loop: no PL samples below %.2f m' % FINAL_HEIGHT_M)
    if result['crossing_mz_m'] is None:
        lines.append('  closed-loop: vehicle never reached %.2f m with the target acquired' % FINAL_HEIGHT_M)
    else:
        lines.append(
            '  at %.2f m gate: horizontal error %.3f m, speed %.2f m/s' % (
                result['crossing_mz_m'], result['crossing_err_m'], result['crossing_speed_ms']))
        lines.append(
            '  braking model from that gate: stock touchdown %.3f m (%s), timed-descent touchdown %.3f m (%s)' % (
                result['sim_stock_err_m'],
                'down' if result['sim_stock_touched'] else 'held',
                result['sim_new_err_m'],
                'down' if result['sim_new_touched'] else 'held'))
    abort = []
    if result['lean_abort']:
        abort.append('lean/attitude (peak lean %.1f deg, peak attitude error %.1f deg)%s' % (
            result['lean_peak_deg'], result['att_err_peak_deg'],
            ', HOLD not climb' if result['lean_hold'] else ', would climb'))
    else:
        abort.append('lean quiet (peak %.1f deg, attitude error %.1f deg)' % (
            result['lean_peak_deg'], result['att_err_peak_deg']))
    if not result['pad_top_known']:
        abort.append('pad top unknown (no altitude while horizontal error < %.2f m)' % PAD_MEMORY_ERR_M)
    elif result['below_pad']:
        abort.append('below pad: %s' % result['below_reason'])
    else:
        abort.append('not below the remembered pad top')
    lines.append('  abort: ' + '; '.join(abort))
    if result['notes']:
        lines.append('  note: ' + result['notes'])
    return '\n'.join(lines)


CSV_FIELDS = [
    'file', 'landing', 'logged_err_m', 'logged_min_mz_m',
    'timed_descent_median_ms', 'stock_descent_median_ms', 'actual_desired_median_ms',
    'held_fraction', 'sim_new_err_m', 'sim_stock_err_m',
    'lean_abort', 'lean_hold', 'below_pad', 'pad_top_known',
    'lean_peak_deg', 'crossing_err_m', 'crossing_speed_ms', 'meas_residual_rms_m', 'notes',
]


def result_row(path, index, result):
    row = {'file': path, 'landing': index, 'notes': result.get('notes', '')}
    for key in CSV_FIELDS:
        if key in ('file', 'landing', 'notes'):
            continue
        row[key] = result.get(key)
    return row


def _msg_get(msg, *names):
    for name in names:
        if hasattr(msg, name):
            value = getattr(msg, name)
            if value is not None:
                return value
    return None


def _load_reader(path):
    try:
        from pymavlink import DFReader
    except ImportError:
        sys.stderr.write('pymavlink is required to read logs (pip install pymavlink)\n')
        return None
    if path.lower().endswith('.log'):
        return DFReader.DFReader_text(path, zero_time_base=False)
    return DFReader.DFReader_binary(path, zero_time_base=False)


def read_log(path):
    '''Return a list of (params, pl, att, alt, rfnd, ctun, land_complete_us) per LAND segment.'''
    reader = _load_reader(path)
    if reader is None:
        return []
    modes = []
    events = []
    pl = []
    att = []
    pos_alt = []
    ctun_alt = []
    rfnd = []
    ctun = []
    params = Params()
    parm_at = []
    try:
        while True:
            msg = reader.recv_msg()
            if msg is None:
                break
            mtype = msg.get_type()
            t_us = int(getattr(msg, 'TimeUS', 0) or 0)
            if mtype == 'PARM':
                params.update(_msg_get(msg, 'Name'), _msg_get(msg, 'Value'))
                # Copy the limits as they stand after this parameter.
                snap = Params()
                snap.__dict__.update(params.__dict__)
                parm_at.append((t_us, snap))
            elif mtype == 'MODE':
                mode_num = _msg_get(msg, 'ModeNum')
                if mode_num is not None:
                    modes.append((t_us, int(mode_num)))
            elif mtype == 'EV':
                ev_id = _msg_get(msg, 'Id')
                if ev_id is not None:
                    events.append((t_us, int(ev_id)))
            elif mtype == 'PL':
                tacq = _msg_get(msg, 'TAcq')
                mz = _as_float(_msg_get(msg, 'mZ', 'MeasZ'))
                px = _as_float(_msg_get(msg, 'pX', 'PosX'))
                py = _as_float(_msg_get(msg, 'pY', 'PosY'))
                vx = _as_float(_msg_get(msg, 'vX', 'VelX'))
                vy = _as_float(_msg_get(msg, 'vY', 'VelY'))
                if None in (px, py, vx, vy, mz):
                    continue
                pl.append({
                    't_us': t_us,
                    'px': px,
                    'py': py,
                    'vx': vx,
                    'vy': vy,
                    'mx': _as_float(_msg_get(msg, 'mX')),
                    'my': _as_float(_msg_get(msg, 'mY')),
                    'mz': mz,
                    'tacq': bool(tacq),
                })
            elif mtype == 'ATT':
                roll = _as_float(_msg_get(msg, 'Roll'))
                pitch = _as_float(_msg_get(msg, 'Pitch'))
                des_roll = _as_float(_msg_get(msg, 'DesRoll'))
                des_pitch = _as_float(_msg_get(msg, 'DesPitch'))
                if None not in (roll, pitch, des_roll, des_pitch):
                    att.append((t_us, (roll, pitch, des_roll, des_pitch)))
            elif mtype == 'POS':
                rel = _as_float(_msg_get(msg, 'RelHomeAlt'))
                if rel is not None:
                    pos_alt.append((t_us, rel))
            elif mtype == 'CTUN':
                alt_ct = _as_float(_msg_get(msg, 'Alt'))
                dcrt = _as_float(_msg_get(msg, 'DCRt'))
                if dcrt is not None:
                    ctun.append((t_us, dcrt))
                if alt_ct is not None:
                    ctun_alt.append((t_us, alt_ct))
            elif mtype == 'RFND':
                dist = _as_float(_msg_get(msg, 'Dist'))
                orient = _msg_get(msg, 'Orient')
                # 25 is ROTATION_PITCH_270, the downward mount. A missing
                # orientation is accepted; any other mount is not the ground.
                if dist is None or dist <= 0.0:
                    continue
                if orient is not None and int(orient) != 25:
                    continue
                rfnd.append((t_us, dist))
    finally:
        reader.close()

    # Baro-only altitude is not a pad reference. POS.RelHomeAlt (EKF) is the
    # coordinate the radar range is subtracted from. CTUN.Alt is the fallback.
    alt = pos_alt if pos_alt else ctun_alt

    if not parm_at:
        parm_at = [(0, params)]

    segments = []
    if any(mode == MODE_LAND for _t, mode in modes):
        for i, (t_us, mode) in enumerate(modes):
            if mode != MODE_LAND:
                continue
            t_end = modes[i + 1][0] if i + 1 < len(modes) else None
            segments.append((t_us, t_end))
    elif pl:
        # No MODE message: split acquired PL into approaches on a 3 s gap.
        gap = int(3.0e6)
        start = pl[0]['t_us']
        prev = start
        for s in pl[1:]:
            if s['t_us'] - prev > gap:
                segments.append((start, prev))
                start = s['t_us']
            prev = s['t_us']
        segments.append((start, None))

    landings = []
    for t0, t1 in segments:
        def _in(t_us, start=t0, end=t1):
            if t_us < start:
                return False
            if end is not None and t_us > end:
                return False
            return True

        seg_pl = [s for s in pl if _in(s['t_us'])]
        if not seg_pl:
            continue
        complete = None
        for t_us, ev_id in events:
            if _in(t_us) and ev_id == EV_LAND_COMPLETE:
                complete = t_us
                break
        snap = params
        for t_us, candidate in parm_at:
            if t_us <= t0:
                snap = candidate
            else:
                break
        landings.append((
            snap,
            seg_pl,
            [a for a in att if _in(a[0])],
            [a for a in alt if _in(a[0])],
            [a for a in rfnd if _in(a[0])],
            [a for a in ctun if _in(a[0])],
            complete,
        ))
    return landings


def iter_logs(paths):
    files = []
    for path in paths:
        if os.path.isdir(path):
            for name in sorted(os.listdir(path)):
                if name.lower().endswith(('.bin', '.log')):
                    files.append(os.path.join(path, name))
        else:
            files.append(path)
    return files


def run_logs(paths, csv_path):
    rows = []
    reports = []
    for path in iter_logs(paths):
        landings = read_log(path)
        if not landings:
            reports.append('%s: no precision-landing segments (need PL, and MODE=LAND or a PL burst)' % path)
            continue
        for index, (params, pl, att, alt, rfnd, ctun, complete) in enumerate(landings, start=1):
            result = analyse_landing(pl, att, alt, rfnd, ctun, params, complete)
            reports.append(format_landing(path, index, result))
            if result.get('usable'):
                rows.append(result_row(path, index, result))
    if csv_path and rows:
        with open(csv_path, 'w', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
            writer.writeheader()
            writer.writerows(rows)
    text = '\n\n'.join(reports)
    if rows:
        n = len(rows)
        lean = sum(1 for r in rows if r['lean_abort'])
        below = sum(1 for r in rows if r['below_pad'])
        text += '\n\n%d landings. Lean abort would have fired on %d. Below-pad abort would have fired on %d.' % (
            n, lean, below)
        text += '\nA lean abort on a landing you were happy with is a threshold to raise before flying it.'
    return text


def _self_test():
    # sqrt_controller matches the piecewise AP_Math curve at a few points.
    # p=1, accel=2.5 -> linear region extends to 2.5 m, output = error.
    if abs(sqrt_controller(1.0, 1.0, 2.5) - 1.0) > 1.0e-6:
        raise SystemExit('sqrt_controller linear region')
    # Past the linear region the sqrt branch is used. error=4, linear_dist=2.5.
    expected = math.sqrt(2.0 * 2.5 * (4.0 - 1.25))
    if abs(sqrt_controller(4.0, 1.0, 2.5) - expected) > 1.0e-6:
        raise SystemExit('sqrt_controller sqrt region')
    # stopping_distance is the inverse.
    if abs(stopping_distance(expected, 1.0, 2.5) - 4.0) > 1.0e-4:
        raise SystemExit('stopping_distance inverse')

    params = Params()
    # Already inside the capture radius: full descent, no hold.
    rate = timed_descent_ms(
        0.2, (0.01, 0.0), (0.0, 0.0), params.land_spd_ms, params.pos_p, params.accel_mss, params.vel_max_ms)
    if abs(rate - params.land_spd_ms) > 1.0e-6:
        raise SystemExit('capture radius should descend at LAND_SPD_MS')

    # Moving away: hold.
    rate = timed_descent_ms(0.3, (0.4, 0.0), (0.3, 0.0), params.land_spd_ms, params.pos_p, params.accel_mss, params.vel_max_ms)
    if rate != 0.0:
        raise SystemExit('moving away should hold, got %s' % rate)

    # Stock slowdown: on top of the target, half land speed; far away, 0.1 m/s.
    on_target = stock_descent_ms(1.0, 0.0, 0.5, 2.5)
    if abs(on_target - 0.25) > 1.0e-6:
        raise SystemExit('stock on-target descent')
    far = stock_descent_ms(1.0, 1.0, 0.5, 2.5)
    if abs(far - 0.1) > 1.0e-6:
        raise SystemExit('stock far descent')

    # Final 0.34 m, 0.6 m off the pin, closing slowly. Full land speed reaches
    # the ground before the braking model is inside the capture radius. The
    # timed law waits, so its touchdown error is smaller.
    err = (0.6, 0.0)
    err_dot = (-0.15, 0.0)
    stock = simulate_touchdown(err, err_dot, 0.34, 0.5, 1.0, 2.5, 10.0, 2.5, False)
    timed = simulate_touchdown(err, err_dot, 0.34, 0.5, 1.0, 2.5, 10.0, 2.5, True)
    if not (timed['err_m'] < stock['err_m'] and timed['err_m'] < 0.08 and stock['err_m'] > 0.2):
        raise SystemExit('sideways/offset case stock %s timed %s' % (stock, timed))

    # A centred, stopped vehicle touches down on the pin under both laws.
    centred = simulate_touchdown((0.0, 0.0), (0.0, 0.0), 0.3, 0.5, 1.0, 2.5, 10.0, 2.5, True)
    if centred['err_m'] > 0.02 or not centred['touched_down']:
        raise SystemExit('centred touchdown %s' % centred)

    # Synthetic log of a calm landing: no lean, radar tracks down to 5 cm,
    # horizontal error stays inside 5 cm. Must not abort.
    samples = []
    t = 0
    for i in range(40):
        mz = 1.2 - i * 0.03
        samples.append({
            't_us': t,
            'px': 0.04,
            'py': 0.0,
            'vx': 0.0,
            'vy': 0.0,
            'mx': 0.04,
            'my': 0.0,
            'mz': mz,
            'tacq': True,
        })
        t += int(0.05e6)
    att = [(s['t_us'], (1.0, -0.5, 0.8, -0.4)) for s in samples]
    # Vehicle altitude follows the radar down onto a pad whose top is at 10 m.
    alt = [(s['t_us'], 10.0 + s['mz']) for s in samples]
    calm = analyse_landing(samples, att, alt, [], [], params, land_complete_us=samples[-1]['t_us'])
    if calm['lean_abort'] or calm['below_pad']:
        raise SystemExit('calm landing aborted: %s' % calm)
    if calm['logged_err_m'] > 0.05:
        raise SystemExit('calm logged error')

    # Same approach, then the vehicle descends 20 cm below the pad with the
    # radar jumping to the ground and the estimate walking off the pin.
    miss = list(samples)
    t = samples[-1]['t_us']
    for i in range(4):
        t += int(0.05e6)
        miss.append({
            't_us': t,
            'px': 0.4,
            'py': 0.0,
            'vx': 0.2,
            'vy': 0.0,
            'mx': 0.4,
            'my': 0.0,
            'mz': 0.05 + 0.3 + i * 0.05,  # range grows: beam left the pin
            'tacq': True,
        })
    miss_alt = [(s['t_us'], 10.0 + min(s['mz'], 1.2)) for s in samples]
    # Continue altitude downward past the pad for the miss samples.
    for i, s in enumerate(miss[len(samples):]):
        miss_alt.append((s['t_us'], 10.0 - 0.05 - i * 0.08))
    missed = analyse_landing(miss, att + [(s['t_us'], (2.0, 1.0, 1.5, 0.5)) for s in miss[len(samples):]],
                             miss_alt, [], [], params, None)
    if not missed['below_pad']:
        raise SystemExit('missed pin was not aborted: %s' % missed)

    # Lean past 10 deg, and past 20 deg, for longer than the persistence window.
    leaned = list(samples)
    lean_att = []
    for i, s in enumerate(leaned):
        if s['mz'] < 1.0:
            roll = 25.0
        else:
            roll = 1.0
        lean_att.append((s['t_us'], (roll, 0.0, 0.0, 0.0)))
    leaned_result = analyse_landing(leaned, lean_att, alt, [], [], params, leaned[-1]['t_us'])
    if not leaned_result['lean_abort'] or not leaned_result['lean_hold']:
        raise SystemExit('lean abort not detected: %s' % leaned_result)

    # Land-complete freezes the check: a post-landing lean must not count.
    settled_att = list(att)
    settled_att.append((samples[-1]['t_us'] + int(1.0e6), (30.0, 30.0, 0.0, 0.0)))
    settled = analyse_landing(samples, settled_att, alt, [], [], params, samples[-1]['t_us'])
    if settled['lean_abort']:
        raise SystemExit('settled lean after land-complete fired an abort')

    print('self-test ok')
    print('offset case: stock touchdown %.3f m, timed touchdown %.3f m' % (stock['err_m'], timed['err_m']))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('logs', nargs='*', help='BIN or LOG files, or directories of them')
    parser.add_argument('--csv', dest='csv_path', help='write one row per landing')
    parser.add_argument('--self-test', action='store_true', help='check the braking model and abort rules')
    args = parser.parse_args()
    if args.self_test:
        _self_test()
        return 0
    if not args.logs:
        parser.error('give at least one log, or --self-test')
    print(run_logs(args.logs, args.csv_path))
    return 0


if __name__ == '__main__':
    sys.exit(main())
