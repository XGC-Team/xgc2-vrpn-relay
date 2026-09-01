"""Vision-pose emit policy (XGC1 PhyMocap).

Decide inside the subscriber callback. Never cache+timer: a held last pose
keeps feeding PX4 after localization dies, adds timer latency, and invites
complex liveness checks.

Target Hz (30, not 50):
  incoming faster  -> keep recent publish times; emit now or drop
  incoming slower  -> emit every accepted frame; no zero-order hold

Emit when any of:
  - first accepted frame
  - wall time since last emit >= 1/target_hz
  - recent window rate (count / span of last N emits) is below target
"""

VISION_EMIT_WINDOW = 5


def should_emit_vision(now_s, published_times, target_hz):
    if target_hz <= 0.0:
        return True
    if not published_times:
        return True
    last_pub_s = published_times[-1]
    dt = now_s - last_pub_s
    if dt < 0.0:
        return True
    if dt >= (1.0 / target_hz):
        return True
    span = now_s - published_times[0]
    if span <= 0.0:
        return False
    return (float(len(published_times)) / span) < target_hz


def remember_vision_emit(now_s, published_times, window=VISION_EMIT_WINDOW):
    published_times.append(now_s)
    overflow = len(published_times) - window
    if overflow > 0:
        del published_times[0:overflow]
