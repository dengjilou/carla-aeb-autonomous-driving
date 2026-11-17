"""Offline metrics, independently reusable on Windows and Linux."""
import math
import statistics


def evaluate(rows, collisions, config):
    if len(rows) < 2:
        raise ValueError("At least two recorded frames are required")
    dt = config["step_s"]
    onset = next((r["time_s"] for r in rows if r["aeb_brake"] > 0), None)
    event_time = next((r["time_s"] for r in rows if r["event_observed"]), None)
    acquired = next((r["time_s"] for r in rows if r["target_id"] == r["lead_id"]), None)
    first_contact = min((e["time_s"] for e in collisions), default=None)
    reasons = []
    if len(rows) != round(config["duration_s"] / dt):
        reasons.append("incomplete_observation")
    if not max(r["sumo_background"] for r in rows):
        reasons.append("missing_sumo_background")
    if event_time is None:
        reasons.append("scenario_event_not_observed")
    if any(e["other_id"] != rows[0]["lead_id"] for e in collisions):
        reasons.append("non_lead_contact")
    if any(abs(r["cross_track_m"]) > 1.0 for r in rows):
        reasons.append("ego_left_reference_lane")
    before_contact = [r for r in rows if first_contact is None or r["time_s"] < first_contact]
    kind = config.get("kind")
    settled = [r for r in before_contact if r["time_s"] >= config["duration_s"] - 5
               and r["target_id"] == r["lead_id"]] if not collisions and kind in ("follow", "slowdown") else []
    errors = [r["gap_m"] - (8.0 + 1.8 * r["ego_speed_mps"]) for r in settled]
    speed_errors = [r["ego_speed_mps"] - r["lead_speed_mps"] for r in settled]
    acc = [(b["ego_speed_mps"] - a["ego_speed_mps"]) / dt for a, b in zip(before_contact, before_contact[1:])]
    jerk = [(b - a) / dt for a, b in zip(acc, acc[1:])]
    # Ignore initial 1 s for comfort metrics. Contact frames are excluded.
    jerk = jerk[max(0, round(1 / dt) - 2):]
    pre_contact = before_contact[-1] if first_contact is not None and before_contact else None
    gaps = [r.get("chord_gap_m",r["gap_m"]) for r in before_contact if r["lateral_clearance_m"] <= 0 and r["longitudinal_m"] >= 0]
    resumed = (kind == "cutout" and event_time is not None and acquired is not None
               and acquired < event_time and rows[-1]["target_id"] != rows[-1]["lead_id"]
               and abs(rows[-1]["ego_speed_mps"] - config["ego_speed_mps"]) < 1)
    other_target = any(r["aeb_brake"] > 0 and r["target_id"] not in (None, r["lead_id"]) for r in rows)
    passed = not collisions and not other_target
    if kind in ("follow", "slowdown", "adjacent"):
        passed = passed and onset is None
    if kind in ("hard_brake", "cutin"):
        passed = passed and acquired is not None
    if kind == "hard_brake":
        passed = passed and onset is not None
    if kind == "cutout":
        passed = passed and resumed and onset is None
    result = {
        "valid": not reasons, "invalid_reasons": reasons,
        "verdict": "invalid" if reasons else ("pass" if passed else "fail"),
        "event_observed_s": event_time, "first_lead_selected_s": acquired,
        "resumed_cruise": resumed if kind == "cutout" else None,
        "collided": bool(collisions), "collision_callbacks": len(collisions),
        "first_collision_s": first_contact, "first_aeb_s": onset,
        "aeb_interventions": int(onset is not None),
        "peak_deceleration_mps2": max(0, -min(acc)) if acc else None,
        "braked_before_contact": onset < first_contact if onset is not None and first_contact is not None else None,
        "pre_contact_speed_kmh": pre_contact["ego_speed_mps"] * 3.6 if pre_contact else None,
        "min_projected_gap_m": min(gaps) if gaps else None,
        "steady_gap_rmse_m": math.sqrt(statistics.mean(x * x for x in errors)) if errors else None,
        "steady_speed_rmse_mps": math.sqrt(statistics.mean(x * x for x in speed_errors)) if speed_errors else None,
        "jerk_rms_mps3": math.sqrt(statistics.mean(x * x for x in jerk)) if jerk else None,
        "final_speed_mps": rows[-1]["ego_speed_mps"],
        "aeb_for_other_target": other_target,
    }
    if before_contact and "path_error_m" in before_contact[0]:
        lateral_errors = [r["path_error_m"] for r in before_contact]
        result.update(path_rmse_m=math.sqrt(statistics.mean(e*e for e in lateral_errors)),
                      path_peak_m=max(abs(e) for e in lateral_errors),
                      heading_peak_rad=max(abs(r["heading_error_rad"]) for r in before_contact))
    if config.get("curved") and before_contact:
        headings=[before_contact[0]["ego_yaw_rad"]]
        for a,b in zip(before_contact,before_contact[1:]):
            delta=b["ego_yaw_rad"]-a["ego_yaw_rad"]
            headings.append(headings[-1]+math.atan2(math.sin(delta),math.cos(delta)))
        turn=math.degrees(max(headings)-min(headings))
        lead_error=max(abs(r["lead_path_error_m"]) for r in before_contact)
        eligible=[r for r in before_contact if 0<r["path_distance_m"]<50]
        coverage=sum(r["target_id"]==r["lead_id"] for r in eligible)/len(eligible) if eligible else 0
        if turn<2:
            reasons.append("insufficient_observed_turn")
        if lead_error>.5:
            reasons.append("lead_left_reference_route")
        if kind in ("follow","slowdown") and coverage<.95:
            passed=False
        result.update(turn_deg=turn,lead_path_peak_m=lead_error,target_coverage=coverage,
                      min_path_gap_m=min(r["path_gap_m"] for r in eligible) if eligible else None,
                      valid=not reasons,invalid_reasons=reasons,
                      verdict="invalid" if reasons else "pass" if passed else "fail")
    return result
