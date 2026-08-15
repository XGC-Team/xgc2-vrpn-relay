"""Vision-pose emit policy.

Target Hz (typical 30):
  incoming faster  -> decide per frame, publish immediately or drop
  incoming slower  -> publish every accepted frame; no zero-order hold
"""


def should_emit_vision(now_s, last_pub_s, target_hz):
    if target_hz <= 0.0 or last_pub_s is None:
        return True
    dt = now_s - last_pub_s
    if dt < 0.0:
        return True
    return dt >= (1.0 / target_hz)
