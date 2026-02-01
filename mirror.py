# import serial, time

# LEADER   = "/dev/cu.usbmodem5A7A0159181"
# FOLLOWER = "/dev/cu.usbmodem5A4B0479901"
# BAUD = 1000000
# IDS  = [1,2,3,4,5,6]

# # Filtering / stability knobs
# ALPHA      = 0.08     # lower = smoother
# DEADBAND   = 12       # ignore tiny changes
# MOVE_MS    = 350      # slower servo motion = less chase
# WRITE_HZ   = 12       # do not spam writes
# MAX_STEP   = 35       # max encoder units per update

# MIN_POS = 0
# MAX_POS = 4095

# def chk(b):
#     return (~(sum(b) & 0xFF)) & 0xFF

# def make_read_pos_pkt(sid):
#     body = [sid, 0x04, 0x02, 0x38, 0x02]  # read present position
#     return bytes([0xFF,0xFF] + body + [chk(body)])

# def make_write_goal_pkt(sid, pos):
#     pos = max(MIN_POS, min(MAX_POS, int(pos)))
#     START_ADDR = 0x2A  # goal position start
#     body = [sid, 0x07, 0x03, START_ADDR,
#             pos & 0xFF, (pos >> 8) & 0xFF,
#             MOVE_MS & 0xFF, (MOVE_MS >> 8) & 0xFF]
#     return bytes([0xFF,0xFF] + body + [chk(body)])

# def read_frame(ser, buf):
#     """Read available bytes, return (frame, buf). Frame is one packet starting with FF FF or None."""
#     n = ser.in_waiting
#     if n:
#         buf += ser.read(n)
#     # frames are short; your replies were 8 bytes
#     while True:
#         i = buf.find(b"\xFF\xFF")
#         if i < 0:
#             return None, buf[-16:]  # keep tail
#         if len(buf) < i + 4:
#             return None, buf[i:]    # need more
#         # Minimal parse: FF FF ID LEN ... (LEN includes instr+params+chk in many variants)
#         sid = buf[i+2]
#         ln  = buf[i+3]
#         # Some replies have LEN=4 => total 4+4=8 bytes (matches your RX)
#         total = 4 + ln
#         if len(buf) < i + total:
#             return None, buf[i:]    # need more
#         frame = buf[i:i+total]
#         buf = buf[i+total:]
#         return frame, buf

# def parse_pos_from_frame(frame, sid):
#     # Expected: FF FF ID LEN ERR POS_L POS_H CHK  (LEN=4)
#     if not frame or len(frame) < 7:
#         return None
#     if frame[0] != 0xFF or frame[1] != 0xFF:
#         return None
#     if frame[2] != sid:
#         return None
#     # position bytes location based on your observed RX: ffff id 04 00 lo hi chk
#     lo = frame[5]
#     hi = frame[6]
#     return lo | (hi << 8)

# def clamp_step(current, target, max_step):
#     if target > current + max_step:
#         return current + max_step
#     if target < current - max_step:
#         return current - max_step
#     return target

# def main():
#     sl = serial.Serial(LEADER, BAUD, timeout=0)    # non-blocking
#     sf = serial.Serial(FOLLOWER, BAUD, timeout=0)  # non-blocking
#     time.sleep(0.2)

#     # Prime follower state
#     follower_pos = {}
#     leader_pos = {}
#     bufL = b""

#     # Initialize follower_pos by reading each joint once
#     for sid in IDS:
#         sl.write(make_read_pos_pkt(sid))
#         t_end = time.time() + 0.1
#         while time.time() < t_end:
#             frame, bufL = read_frame(sl, bufL)
#             if frame:
#                 p = parse_pos_from_frame(frame, sid)
#                 if p is not None:
#                     follower_pos[sid] = p
#                     leader_pos[sid] = p
#                     break

#     last_write = {sid: 0.0 for sid in IDS}

#     print("Mirroring leader -> follower (stable). Ctrl+C to stop.")
#     i = 0
#     try:
#         while True:
#             sid = IDS[i % len(IDS)]
#             i += 1

#             # Request one joint read
#             sl.write(make_read_pos_pkt(sid))

#             # Pull frames for a short window
#             t_end = time.time() + 0.01
#             while time.time() < t_end:
#                 frame, bufL = read_frame(sl, bufL)
#                 if not frame:
#                     break
#                 # accept any sid frame; keep latest
#                 fsid = frame[2]
#                 if fsid in IDS:
#                     p = parse_pos_from_frame(frame, fsid)
#                     if p is not None:
#                         leader_pos[fsid] = p

#             if sid not in leader_pos or sid not in follower_pos:
#                 time.sleep(0.005)
#                 continue

#             raw = leader_pos[sid]
#             cur = follower_pos[sid]

#             # Deadband
#             if abs(raw - cur) < DEADBAND:
#                 time.sleep(0.005)
#                 continue

#             # Low-pass toward leader
#             tgt = int((1 - ALPHA) * cur + ALPHA * raw)

#             # Step clamp to avoid twitch
#             tgt = clamp_step(cur, tgt, MAX_STEP)

#             # Rate limit writes
#             now = time.time()
#             if now - last_write[sid] >= (1.0 / WRITE_HZ):
#                 sf.write(make_write_goal_pkt(sid, tgt))
#                 follower_pos[sid] = tgt
#                 last_write[sid] = now

#             time.sleep(0.005)

#     finally:
#         sl.close()
#         sf.close()

# if __name__ == "__main__":
#     main()

#=======================================================================================================================================================
#Above kinda sorta works
#=======================================================================================================================================================

# import serial, time

# LEADER   = "/dev/cu.usbmodem5A7A0159181"
# FOLLOWER = "/dev/cu.usbmodem5A4B0479901"
# BAUD = 1000000
# IDS  = [1,2,3,4,5,6]

# # Stability knobs
# ALPHA        = 0.08   # leader -> follower low-pass (smaller = smoother)
# HOLD_BAND    = 25     # don't command tiny moves (kills end-of-move jerk)
# MOVE_MS      = 600    # keep servo "in motion" longer (reduces snap)
# WRITE_HZ     = 12     # rate limit writes per joint
# MAX_STEP     = 35     # max encoder units per update
# FOLLOW_ALPHA = 0.7    # follower-side inertia (larger = smoother)

# MIN_POS = 0
# MAX_POS = 4095

# def chk(b):
#     return (~(sum(b) & 0xFF)) & 0xFF

# def make_read_pos_pkt(sid):
#     body = [sid, 0x04, 0x02, 0x38, 0x02]  # read present position
#     return bytes([0xFF, 0xFF] + body + [chk(body)])

# def make_write_goal_pkt(sid, pos):
#     pos = max(MIN_POS, min(MAX_POS, int(pos)))
#     START_ADDR = 0x2A  # goal position start: posL,posH,timeL,timeH
#     body = [
#         sid, 0x07, 0x03, START_ADDR,
#         pos & 0xFF, (pos >> 8) & 0xFF,
#         MOVE_MS & 0xFF, (MOVE_MS >> 8) & 0xFF
#     ]
#     return bytes([0xFF, 0xFF] + body + [chk(body)])

# def read_frame(ser, buf):
#     n = ser.in_waiting
#     if n:
#         buf += ser.read(n)
#     while True:
#         i = buf.find(b"\xFF\xFF")
#         if i < 0:
#             return None, buf[-16:]
#         if len(buf) < i + 4:
#             return None, buf[i:]
#         sid = buf[i+2]
#         ln  = buf[i+3]
#         total = 4 + ln  # with LEN=4, total=8 (matches your observed RX)
#         if len(buf) < i + total:
#             return None, buf[i:]
#         frame = buf[i:i+total]
#         buf = buf[i+total:]
#         return frame, buf

# def parse_pos_from_frame(frame, sid):
#     # Expected: FF FF ID LEN ERR POS_L POS_H CHK  (LEN=4)
#     if not frame or len(frame) < 7:
#         return None
#     if frame[0] != 0xFF or frame[1] != 0xFF:
#         return None
#     if frame[2] != sid:
#         return None
#     lo = frame[5]
#     hi = frame[6]
#     return lo | (hi << 8)

# def clamp_step(current, target, max_step):
#     if target > current + max_step:
#         return current + max_step
#     if target < current - max_step:
#         return current - max_step
#     return target

# def main():
#     sl = serial.Serial(LEADER, BAUD, timeout=0)    # non-blocking
#     sf = serial.Serial(FOLLOWER, BAUD, timeout=0)  # non-blocking
#     time.sleep(0.2)

#     follower_pos = {}
#     leader_pos = {}
#     bufL = b""

#     # Prime state: read each joint once
#     for sid in IDS:
#         sl.write(make_read_pos_pkt(sid))
#         t_end = time.time() + 0.15
#         while time.time() < t_end:
#             frame, bufL = read_frame(sl, bufL)
#             if frame:
#                 fsid = frame[2]
#                 if fsid == sid:
#                     p = parse_pos_from_frame(frame, sid)
#                     if p is not None:
#                         follower_pos[sid] = p
#                         leader_pos[sid] = p
#                         break

#     last_write = {sid: 0.0 for sid in IDS}

#     print("Mirroring leader -> follower (reduced end-of-move jerk). Ctrl+C to stop.")
#     i = 0
#     try:
#         while True:
#             sid = IDS[i % len(IDS)]
#             i += 1

#             # Request read for one joint
#             sl.write(make_read_pos_pkt(sid))

#             # Pull frames briefly; keep latest for any joint
#             t_end = time.time() + 0.01
#             while time.time() < t_end:
#                 frame, bufL = read_frame(sl, bufL)
#                 if not frame:
#                     break
#                 fsid = frame[2]
#                 if fsid in IDS:
#                     p = parse_pos_from_frame(frame, fsid)
#                     if p is not None:
#                         leader_pos[fsid] = p

#             if sid not in leader_pos or sid not in follower_pos:
#                 time.sleep(0.005)
#                 continue

#             raw = leader_pos[sid]
#             cur = follower_pos[sid]

#             # Hold band: do nothing for small errors
#             if abs(raw - cur) < HOLD_BAND:
#                 time.sleep(0.005)
#                 continue

#             # Smooth toward leader
#             tgt = int((1 - ALPHA) * cur + ALPHA * raw)

#             # Prevent twitch
#             tgt = clamp_step(cur, tgt, MAX_STEP)

#             # Follower-side inertia (prevents end-of-move snap)
#             tgt = int(FOLLOW_ALPHA * cur + (1 - FOLLOW_ALPHA) * tgt)

#             # Rate limit writes
#             now = time.time()
#             if now - last_write[sid] >= (1.0 / WRITE_HZ):
#                 sf.write(make_write_goal_pkt(sid, tgt))
#                 follower_pos[sid] = tgt
#                 last_write[sid] = now

#             time.sleep(0.005)

#     finally:
#         sl.close()
#         sf.close()

# if __name__ == "__main__":
#     main()

#=======================================================================================================================================================
#Above is more precise and forcibly conttrolled but extermely slow, not the worst but not the best
#=======================================================================================================================================================
# import serial, time

# LEADER   = "/dev/cu.usbmodem5A7A0159181"
# FOLLOWER = "/dev/cu.usbmodem5A4B0479901"
# BAUD = 1000000
# IDS  = [1,2,3,4,5,6]

# # FAST SETTINGS
# MOVE_MS   = 40        # fast servo motion
# LOOP_DT   = 0.01      # ~100 Hz outer loop (practically you’ll get less)
# DEADBAND  = 10         # do nothing if leader didn’t change
# MAX_JUMP  = 300       # ignore absurd single-step jumps (bad read)

# MIN_POS = 0
# MAX_POS = 4095

# def chk(b):
#     return (~(sum(b) & 0xFF)) & 0xFF

# def make_read_pos_pkt(sid):
#     body = [sid, 0x04, 0x02, 0x38, 0x02]  # read present position
#     return bytes([0xFF,0xFF] + body + [chk(body)])

# def make_write_goal_pkt(sid, pos):
#     pos = max(MIN_POS, min(MAX_POS, int(pos)))
#     START_ADDR = 0x2A  # goal position start: posL,posH,timeL,timeH
#     body = [
#         sid, 0x07, 0x03, START_ADDR,
#         pos & 0xFF, (pos >> 8) & 0xFF,
#         MOVE_MS & 0xFF, (MOVE_MS >> 8) & 0xFF
#     ]
#     return bytes([0xFF,0xFF] + body + [chk(body)])

# def read_exact_frame(ser, sid, max_wait=0.01):
#     """
#     Read one reply frame for sid: FF FF ID LEN ERR POS_L POS_H CHK (typically 8 bytes, LEN=4).
#     Robust against partial reads / misalignment.
#     """
#     deadline = time.time() + max_wait
#     buf = b""

#     while time.time() < deadline:
#         # pull whatever is available
#         n = ser.in_waiting
#         if n:
#             buf += ser.read(n)
#         else:
#             # small yield so device can respond
#             time.sleep(0.0005)

#         # find header
#         i = buf.find(b"\xFF\xFF")
#         if i < 0:
#             # keep tail small
#             buf = buf[-16:]
#             continue

#         # need at least header+id+len
#         if len(buf) < i + 4:
#             continue

#         if buf[i+2] != sid:
#             # discard this header, keep scanning
#             buf = buf[i+2:]
#             continue

#         ln = buf[i+3]
#         total = 4 + ln
#         if len(buf) < i + total:
#             continue

#         frame = buf[i:i+total]
#         # minimal sanity
#         if len(frame) >= 7:
#             # position bytes in your observed format: ... ERR POS_L POS_H ...
#             pos = frame[5] | (frame[6] << 8)
#             return pos

#         buf = buf[i+total:]

#     return None

# def main():
#     sl = serial.Serial(LEADER, BAUD, timeout=0)    # non-blocking; we manage timing
#     sf = serial.Serial(FOLLOWER, BAUD, timeout=0)
#     time.sleep(0.2)

#     # Flush once at start so we don't parse old bytes
#     sl.reset_input_buffer()
#     sf.reset_input_buffer()

#     # Initialize follower to current follower pose and "arm" the filter
#     last = {}
#     for sid in IDS:
#         sl.write(make_read_pos_pkt(sid))
#         p = read_exact_frame(sl, sid, max_wait=0.03)
#         if p is not None:
#             last[sid] = p

#     print("FAST+SAFE mirror active. Move leader; follower will track. Ctrl+C to stop.")

#     try:
#         while True:
#             t0 = time.time()

#             for sid in IDS:
#                 sl.write(make_read_pos_pkt(sid))
#                 p = read_exact_frame(sl, sid, max_wait=0.01)
#                 if p is None:
#                     continue

#                 lp = last.get(sid, p)

#                 # ignore tiny noise
#                 if abs(p - lp) < DEADBAND:
#                     continue

#                 # ignore absurd jumps (bad read / wrap)
#                 if abs(p - lp) > MAX_JUMP:
#                     continue

#                 last[sid] = p
#                 sf.write(make_write_goal_pkt(sid, p))

#             dt = time.time() - t0
#             time.sleep(max(0.0, LOOP_DT - dt))

#     finally:
#         sl.close()
#         sf.close()

# if __name__ == "__main__":
#     main()
####################################################################
#Above is Perfect
####################################################################
import serial, time

LEADER   = "/dev/cu.usbmodem5A7A0159181"
FOLLOWER = "/dev/cu.usbmodem5A4B0479901"
BAUD = 1000000
IDS  = [1,2,3,4,5,6]

# FAST SETTINGS
MOVE_MS  = 40
LOOP_DT  = 0.01
DEADBAND = 6
MAX_JUMP = 300

# Safety
MAX_CMD_STEP = 120

# -------------------------
# LIMIT SET 1: NATURAL
# (+5/-5 margin already applied)
JOINT_LIMITS_NATURAL = {
  1: (943, 3517),
  2: (872, 3277),
  3: (858, 3105),
  4: (974, 3338),
  5: (204, 4013),
  6: (1661, 3153),
}

# LIMIT SET 2: COFFEE POURING
# (only joint 6 tightened for cup workspace)
JOINT_LIMITS_COFFEE = {
  1: (943, 3517),
  2: (872, 3277),
  3: (858, 3105),
  4: (974, 3338),
  5: (204, 4013),
  6: (2039, 3153),
}

# Choose ONE by commenting/uncommenting:
#JOINT_LIMITS = JOINT_LIMITS_NATURAL
JOINT_LIMITS = JOINT_LIMITS_COFFEE
# -------------------------

MIN_POS = 0
MAX_POS = 4095

def chk(b):
    return (~(sum(b) & 0xFF)) & 0xFF

def make_read_pos_pkt(sid):
    body = [sid, 0x04, 0x02, 0x38, 0x02]
    return bytes([0xFF,0xFF] + body + [chk(body)])

def make_write_goal_pkt(sid, pos):
    pos = max(MIN_POS, min(MAX_POS, int(pos)))
    body = [
        sid, 0x07, 0x03, 0x2A,
        pos & 0xFF, (pos >> 8) & 0xFF,
        MOVE_MS & 0xFF, (MOVE_MS >> 8) & 0xFF
    ]
    return bytes([0xFF,0xFF] + body + [chk(body)])

def clamp_to_limits(sid, pos):
    lo, hi = JOINT_LIMITS[sid]
    return max(lo, min(hi, pos))

def read_exact_frame(ser, sid, max_wait=0.01):
    deadline = time.time() + max_wait
    buf = b""

    while time.time() < deadline:
        n = ser.in_waiting
        if n:
            buf += ser.read(n)
        else:
            time.sleep(0.0005)

        i = buf.find(b"\xFF\xFF")
        if i < 0:
            buf = buf[-16:]
            continue

        if len(buf) < i + 4:
            continue

        if buf[i+2] != sid:
            buf = buf[i+2:]
            continue

        ln = buf[i+3]
        total = 4 + ln
        if len(buf) < i + total:
            continue

        frame = buf[i:i+total]
        if len(frame) >= 7:
            return frame[5] | (frame[6] << 8)

        buf = buf[i+total:]

    return None

def main():
    sl = serial.Serial(LEADER, BAUD, timeout=0)
    sf = serial.Serial(FOLLOWER, BAUD, timeout=0)
    time.sleep(0.2)

    sl.reset_input_buffer()
    sf.reset_input_buffer()

    # Initialize follower state
    last = {}
    for sid in IDS:
        sl.write(make_read_pos_pkt(sid))
        p = read_exact_frame(sl, sid, max_wait=0.03)
        if p is not None:
            last[sid] = p

    print("Mirror active with selectable joint limits. Ctrl+C to stop.")

    try:
        while True:
            t0 = time.time()

            for sid in IDS:
                sl.write(make_read_pos_pkt(sid))
                p = read_exact_frame(sl, sid, max_wait=0.01)
                if p is None:
                    continue

                lp = last.get(sid, p)

                if abs(p - lp) < DEADBAND:
                    continue
                if abs(p - lp) > MAX_JUMP:
                    continue

                p = clamp_to_limits(sid, p)
                p = max(lp - MAX_CMD_STEP, min(lp + MAX_CMD_STEP, p))

                last[sid] = p
                sf.write(make_write_goal_pkt(sid, p))

            dt = time.time() - t0
            time.sleep(max(0.0, LOOP_DT - dt))

    finally:
        sl.close()
        sf.close()

if __name__ == "__main__":
    main()
