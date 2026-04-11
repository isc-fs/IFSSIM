"""
FSG Scoring Formulas (2024 Rules)
Calculates corrected times and scores for each dynamic event.
"""


def corrected_time_trackdrive(elapsed: float, doo: int, oc: int) -> float:
    """Trackdrive: T_corrected = T_elapsed + DOO × 2.0 + OC × 10.0"""
    return elapsed + doo * 2.0 + oc * 10.0


def corrected_time_autocross(elapsed: float, doo: int, oc: int) -> float:
    """Autocross: same formula as trackdrive"""
    return elapsed + doo * 2.0 + oc * 10.0


def corrected_time_acceleration(elapsed: float, doo: int, oc: int) -> float:
    """Acceleration: T_corrected = T_elapsed + DOO × 2.0 + OC × 10.0"""
    return elapsed + doo * 2.0 + oc * 10.0


def corrected_time_skidpad(right_laps: list, left_laps: list, doo: int) -> float:
    """Skidpad: T_corrected = avg(right) + avg(left) + DOO × 0.2"""
    avg_right = sum(right_laps) / len(right_laps) if right_laps else 0
    avg_left = sum(left_laps) / len(left_laps) if left_laps else 0
    return avg_right + avg_left + doo * 0.2


def score_dynamic(t_corrected: float, t_best: float, max_points: float) -> float:
    """Generic dynamic event score: max(0, max_points × (1.5 × T_best / T_corrected - 0.5))"""
    if t_corrected <= 0 or t_best <= 0:
        return 0.0
    return max(0.0, max_points * (1.5 * t_best / t_corrected - 0.5))


def compute_scoring(event: str, lap_times: list, doo: int, oc: int,
                    t_best: float = None) -> dict:
    """
    Compute full scoring breakdown for an event.

    Args:
        event: "trackdrive", "autocross", "acceleration", "skidpad"
        lap_times: list of lap times in seconds
        doo: cone hit count
        oc: off-track count
        t_best: best known time for normalization (optional)

    Returns:
        dict with scoring breakdown
    """
    total_elapsed = sum(lap_times) if lap_times else 0.0
    best_lap = min(lap_times) if lap_times else 0.0

    if event == "trackdrive":
        t_corrected = corrected_time_trackdrive(total_elapsed, doo, oc)
        max_points = 200.0
    elif event == "autocross":
        t_corrected = corrected_time_autocross(total_elapsed, doo, oc)
        max_points = 100.0
    elif event == "acceleration":
        t_corrected = corrected_time_acceleration(total_elapsed, doo, oc)
        max_points = 75.0
    elif event == "skidpad":
        # Split laps: first 2 = right, last 2 = left
        right = lap_times[:2] if len(lap_times) >= 2 else lap_times
        left = lap_times[2:4] if len(lap_times) >= 4 else []
        t_corrected = corrected_time_skidpad(right, left, doo)
        max_points = 75.0
    else:
        t_corrected = total_elapsed
        max_points = 0.0

    if t_best is None:
        t_best = best_lap if best_lap > 0 else t_corrected

    score = score_dynamic(t_corrected, t_best, max_points)

    return {
        "event": event,
        "laps_completed": len(lap_times),
        "lap_times": lap_times,
        "best_lap": round(best_lap, 3),
        "total_elapsed": round(total_elapsed, 3),
        "doo_count": doo,
        "doo_penalty_s": round(doo * 2.0, 1) if event != "skidpad" else round(doo * 0.2, 1),
        "oc_count": oc,
        "oc_penalty_s": round(oc * 10.0, 1) if event != "skidpad" else 0.0,
        "corrected_time": round(t_corrected, 3),
        "t_best_reference": round(t_best, 3),
        "max_points": max_points,
        "score": round(score, 1),
    }
