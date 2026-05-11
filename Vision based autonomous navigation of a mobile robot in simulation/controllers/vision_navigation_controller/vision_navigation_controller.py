"""
======================================================================================
Hybrid Autonomous Navigation Controller for E-puck (Webots) + Floor Plane Segmentation 
======================================================================================
"""

import math
import numpy as np
import cv2
from controller import Robot, Camera, Motor, GPS, Compass

# ════════════════════════════════════════════════════════════════════════════
#  TUNABLE PARAMETERS
# ════════════════════════════════════════════════════════════════════════════

TIME_STEP         = 32
GOAL_X            = 2.5
GOAL_Y            = -0.5
GOAL_TOLERANCE_M  = 0.01
MAX_SPEED         = 6.28
FORWARD_SPEED     = 0.75 * MAX_SPEED
STEER_SPEED       = 0.35 * MAX_SPEED
TURN_SPEED        = 0.55 * MAX_SPEED
HEADING_KP        = 1.6
HEADING_DEADBAND  = 0.08
LARGE_HEADING_ERR = 0.50
OBS_THRESHOLD     = 90           # legacy, used only in draw_debug comparison window
TURN_DURATION     = 20
STRIDE_DURATION   = 50
AVOID_COOLDOWN    = 12
SLOWDOWN_RADIUS   = 0.10
MIN_SPEED_FRAC    = 0.25
WARMUP_STEPS      = 3
SHOW_DEBUG_WINDOWS = True

# ── Floor Model Parameters (unchanged from v1) ───────────────────────────────
FLOOR_SAMPLE_H_FRAC  = 0.30
FLOOR_SAMPLE_W_FRAC  = 0.60
MAHAL_THRESHOLD      = 2.0
FLOOR_LEARN_ALPHA    = 0.08
OBSTACLE_ZONE_H_FRAC = 0.60
BLOCKED_PIXEL_RATIO  = 0.10
MIN_COVARIANCE_EIG   = 0.5

# ── FIX-1a / FIX-A: Stride Interrupt Parameters ─────────────────────────────
# STRIDE_INTERRUPT_TURN_DURATION: how long the robot turns after a stride
#   interrupt. Typically shorter than TURN_DURATION because the robot is
#   already partially around the obstacle.
#   Tune: raise if robot clips the second obstacle; lower if it over-rotates.
STRIDE_INTERRUPT_TURN_DURATION = 12   # steps (~384 ms at TIME_STEP=32)

# FIX-A: STRIDE_INTERRUPT_COOLDOWN must be STRICTLY GREATER than
#   STRIDE_INTERRUPT_TURN_DURATION.
#
#   ROOT CAUSE of oscillation (BUG 3):
#     Old value was 8 < 12 (turn duration). The cooldown expired 4 steps
#     BEFORE the turn finished. On stride step 1, the obstacle was still
#     centered and cooldown==0, so an interrupt fired immediately — infinite loop.
#
#   Fix: set cooldown = turn_duration + 4, so it expires 4 steps INTO the stride.
#   By then the robot has moved forward and the obstacle has shifted in the FOV.
#   Invariant to maintain: STRIDE_INTERRUPT_COOLDOWN > STRIDE_INTERRUPT_TURN_DURATION
STRIDE_INTERRUPT_COOLDOWN = STRIDE_INTERRUPT_TURN_DURATION + 4   # = 16 steps

# FIX-B: Escalating turn duration for repeated interrupts at the same obstacle.
#   Each consecutive interrupt (without an intervening clean stride) adds this
#   many extra steps to the interrupt turn, up to MAX_INTERRUPT_TURN_DURATION.
#   This prevents wall-hugging where short turns are insufficient to gain
#   angular clearance from a wide obstacle (e.g. 0.35 m orange wall).
INTERRUPT_ESCALATION_STEP    = 4     # extra steps per consecutive interrupt
MAX_INTERRUPT_TURN_DURATION  = TURN_DURATION   # cap at full avoidance turn (20)

# FIX-C: Position stall detection.
#   If the robot's displacement from its avoidance-entry position is below
#   STALL_RADIUS after STALL_TRIGGER consecutive interrupts, force a full
#   TURN_DURATION escape turn and reset the stall counter.
STALL_RADIUS   = 0.06   # metres — less than this = considered stalled
STALL_TRIGGER  = 5      # consecutive interrupts before forced escape
# ── end FIX-1a / FIX-A ───────────────────────────────────────────────────────


# ════════════════════════════════════════════════════════════════════════════
#  INITIALISATION
# ════════════════════════════════════════════════════════════════════════════

robot = Robot()

camera      = robot.getDevice("camera");  camera.enable(TIME_STEP)
gps         = robot.getDevice("gps");     gps.enable(TIME_STEP)
compass     = robot.getDevice("compass"); compass.enable(TIME_STEP)
left_motor  = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

CAM_W = camera.getWidth()
CAM_H = camera.getHeight()

for m in (left_motor, right_motor):
    m.setPosition(float('inf'))
    m.setVelocity(0.0)


# ════════════════════════════════════════════════════════════════════════════
#  HELPERS  (unchanged)
# ════════════════════════════════════════════════════════════════════════════

def set_motors(lv, rv):
    left_motor.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, lv)))
    right_motor.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, rv)))

def get_position():
    v = gps.getValues()
    return float(v[0]), float(v[1])

def get_bearing():
    c = compass.getValues()
    return math.atan2(c[0], c[1])

def angle_to_goal(rx, ry):
    return math.atan2(GOAL_Y - ry, GOAL_X - rx)

def normalize_angle(a):
    while a >  math.pi: a -= 2.0 * math.pi
    while a <= -math.pi: a += 2.0 * math.pi
    return a

def distance_to_goal(rx, ry):
    return math.hypot(GOAL_X - rx, GOAL_Y - ry)


# ════════════════════════════════════════════════════════════════════════════
#  FLOOR MODEL CLASS  (unchanged from v1)
# ════════════════════════════════════════════════════════════════════════════

class FloorModel:
    """
    Adaptive floor colour model in CIE-LAB space.
    Detects non-floor pixels via per-pixel Mahalanobis distance.
    See v1 docstring for full design rationale.
    """

    def __init__(self):
        self.mean    = np.zeros(3, dtype=np.float32)
        self.cov_inv = np.eye(3,  dtype=np.float32)
        self.ready   = False
        print("[FloorModel] Initialised — waiting for first floor sample.")

    @staticmethod
    def _sample_region():
        row_start  = int(CAM_H * (1.0 - FLOOR_SAMPLE_H_FRAC))
        col_margin = int(CAM_W * (1.0 - FLOOR_SAMPLE_W_FRAC) / 2)
        return row_start, CAM_H, col_margin, CAM_W - col_margin

    def update(self, bgr):
        """
        Update the floor colour model from the current frame.
        NOTE (FIX-2): This method must only be called during STATE_SEEK.
        Calling it during AVOID_TURN or AVOID_DRIVE risks contaminating the
        model with obstacle colours from the lateral sample strip.
        """
        r0, r1, c0, c1 = self._sample_region()
        patch_lab = cv2.cvtColor(bgr[r0:r1, c0:c1],
                                 cv2.COLOR_BGR2Lab).reshape(-1, 3).astype(np.float32)

        new_mean = patch_lab.mean(axis=0)
        centered = patch_lab - new_mean
        new_cov  = (centered.T @ centered) / max(len(patch_lab) - 1, 1)
        new_cov += np.eye(3, dtype=np.float32) * MIN_COVARIANCE_EIG

        if not self.ready:
            self.mean = new_mean
            self._set_cov(new_cov)
            self.ready = True
            print(f"[FloorModel] Bootstrap  mean_LAB={self.mean.round(1)}  "
                  f"patch_size={bgr[r0:r1, c0:c1].shape[:2]}")
        else:
            self.mean = ((1.0 - FLOOR_LEARN_ALPHA) * self.mean
                         + FLOOR_LEARN_ALPHA * new_mean)
            blended = ((1.0 - FLOOR_LEARN_ALPHA) * np.linalg.inv(self.cov_inv)
                       + FLOOR_LEARN_ALPHA * new_cov)
            self._set_cov(blended)

    def _set_cov(self, cov):
        try:
            self.cov_inv = np.linalg.inv(cov)
        except np.linalg.LinAlgError:
            print("[FloorModel] WARNING: Singular covariance — using identity.")
            self.cov_inv = np.eye(3, dtype=np.float32)

    def obstacle_mask(self, bgr):
        """
        Returns uint8 mask (CAM_H × CAM_W):  255 = obstacle,  0 = floor.
        Top OBSTACLE_ZONE_H_FRAC rows are zeroed out (background suppression).
        """
        if not self.ready:
            return np.zeros((CAM_H, CAM_W), dtype=np.uint8)

        lab  = cv2.cvtColor(bgr, cv2.COLOR_BGR2Lab).astype(np.float32)
        diff = lab - self.mean
        tmp  = diff @ self.cov_inv
        dist = np.sqrt(np.einsum('hwc,hwc->hw', tmp, diff))

        mask = (dist > MAHAL_THRESHOLD).astype(np.uint8) * 255
        mask[: int(CAM_H * OBSTACLE_ZONE_H_FRAC), :] = 0
        return mask

    def zone_blocked(self, mask):
        """
        Returns (left_blocked, center_blocked, right_blocked).
        A zone is blocked when obstacle-pixel fraction > BLOCKED_PIXEL_RATIO.
        """
        zones = [mask[:, : CAM_W//3],
                 mask[:, CAM_W//3 : 2*CAM_W//3],
                 mask[:, 2*CAM_W//3 :]]
        return tuple(np.count_nonzero(z) / z.size > BLOCKED_PIXEL_RATIO
                     for z in zones)


# ════════════════════════════════════════════════════════════════════════════
#  CAMERA PROCESSING
# ════════════════════════════════════════════════════════════════════════════

def process_camera_floor_model(floor_model, update_model: bool):
    """
    Capture and process one camera frame.

    Parameters
    ----------
    floor_model  : FloorModel instance
    update_model : bool
        # FIX-2: Pass True only during STATE_SEEK.
        During avoidance states this is False, freezing the floor model
        and preventing obstacle-colour contamination of the floor baseline.

    Returns
    -------
    lm, cm, rm          : float  — legacy zone brightness means (debug only)
    left_b, ctr_b, r_b  : bool   — zone blockage flags from floor model
    gray_u8             : ndarray — grayscale frame (for debug)
    frame_bgr           : ndarray — raw BGR frame
    obs_mask            : ndarray — binary obstacle mask
    """
    raw = camera.getImage()
    img = np.frombuffer(raw, dtype=np.uint8).reshape((CAM_H, CAM_W, 4))
    bgr = img[:, :, :3].copy()

    # Legacy brightness means (debug display only)
    gray = np.mean(bgr, axis=2).astype(np.float32)
    wt   = np.ones((CAM_H, CAM_W), dtype=np.float32)
    wt[: CAM_H // 3, :] = 0.4
    lm = float(np.average(gray[:, : CAM_W//3],          weights=wt[:, : CAM_W//3]))
    cm = float(np.average(gray[:, CAM_W//3:2*CAM_W//3], weights=wt[:, CAM_W//3:2*CAM_W//3]))
    rm = float(np.average(gray[:, 2*CAM_W//3:],         weights=wt[:, 2*CAM_W//3:]))

    # FIX-2: Only update the floor model when explicitly told to (STATE_SEEK only)
    if update_model:
        floor_model.update(bgr)

    obs_mask              = floor_model.obstacle_mask(bgr)
    left_b, ctr_b, right_b = floor_model.zone_blocked(obs_mask)

    return lm, cm, rm, left_b, ctr_b, right_b, gray.astype(np.uint8), bgr, obs_mask


# ════════════════════════════════════════════════════════════════════════════
#  DEBUG VISUALISATION
# ════════════════════════════════════════════════════════════════════════════

def draw_debug(frame_bgr, gray_u8, obs_mask, lm, cm, rm,
               label, dist, herr_deg, stride_interrupted=False):
    """
    Three debug windows:
      1. Camera View  — live frame + obstacle overlay + zone dividers
      2. Floor Model Mask  — Mahalanobis distance binary mask
      3. Legacy Mask  — original grayscale threshold mask (comparison)

    stride_interrupted: bool
        # FIX-6: When True, renders a MAGENTA border to signal stride interrupt.
    """
    dbg = frame_bgr.copy()

    dbg[obs_mask > 0] = [0, 0, 255]

    # FIX-6: Visual indicator when stride was interrupted
    if stride_interrupted:
        cv2.rectangle(dbg, (0, 0), (CAM_W - 1, CAM_H - 1), (255, 0, 255), 4)

    cv2.line(dbg, (CAM_W//3, 0),   (CAM_W//3, CAM_H),   (0, 0, 255), 2)
    cv2.line(dbg, (2*CAM_W//3, 0), (2*CAM_W//3, CAM_H), (0, 0, 255), 2)

    # Floor sample region box (green)
    rs   = int(CAM_H * (1.0 - FLOOR_SAMPLE_H_FRAC))
    cm_l = int(CAM_W * (1.0 - FLOOR_SAMPLE_W_FRAC) / 2)
    cm_r = CAM_W - cm_l
    cv2.rectangle(dbg, (cm_l, rs), (cm_r, CAM_H - 1), (0, 255, 0), 1)

    cv2.putText(dbg, f"L:{lm:.0f}", (4, 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)
    cv2.putText(dbg, f"C:{cm:.0f}", (CAM_W//3+4, 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)
    cv2.putText(dbg, f"R:{rm:.0f}", (2*CAM_W//3+4, 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)
    cv2.putText(dbg, label, (4, CAM_H - 32),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 0), 2)
    cv2.putText(dbg, f"dist:{dist:.2f}m  herr:{herr_deg:.1f}d",
                (4, CAM_H - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200, 200, 0), 1)

    cv2.imshow("Robot Camera View", dbg)
    cv2.imshow("Floor Model Mask",  obs_mask)

    _, legacy_mask = cv2.threshold(gray_u8, OBS_THRESHOLD, 255, cv2.THRESH_BINARY_INV)
    cv2.imshow("Legacy Mask (gray threshold)", legacy_mask)

    cv2.waitKey(1)


# ════════════════════════════════════════════════════════════════════════════
#  STATE MACHINE SETUP
# ════════════════════════════════════════════════════════════════════════════

STATE_SEEK        = "SEEK_GOAL"
STATE_AVOID_TURN  = "AVOID_TURN"
STATE_AVOID_DRIVE = "AVOID_DRIVE"
STATE_ARRIVED     = "ARRIVED"

current_state    = STATE_SEEK
avoid_direction  = 0
avoid_counter    = 0
cooldown_counter = 0
step_count       = 0

# FIX-5: New variable — tracks post-interrupt cooldown steps
stride_interrupt_cooldown = 0

# FIX-6: Flag that persists for one display frame to show interrupt indicator
stride_just_interrupted = False

# FIX-D: New state variables for escalation (FIX-B) and stall detection (FIX-C)
interrupt_consecutive = 0    # counts interrupts without a clean stride between them
stall_count           = 0    # counts interrupts without meaningful position change
stall_anchor          = None # (x, y) position recorded at avoidance entry

floor_model = FloorModel()

print(f"[NAV] Goal=({GOAL_X}, {GOAL_Y}) m   tolerance={GOAL_TOLERANCE_M} m")
print(f"[NAV] Stride blind-spot fix: ACTIVE  "
      f"(interrupt cooldown={STRIDE_INTERRUPT_COOLDOWN} steps, "
      f"interrupt turn={STRIDE_INTERRUPT_TURN_DURATION} steps)")
print(f"[NAV] Oscillation fix: ACTIVE  "
      f"(escalation={INTERRUPT_ESCALATION_STEP}/interrupt, "
      f"stall_trigger={STALL_TRIGGER} interrupts, stall_radius={STALL_RADIUS} m)")


# ════════════════════════════════════════════════════════════════════════════
#  MAIN LOOP
# ════════════════════════════════════════════════════════════════════════════

while robot.step(TIME_STEP) != -1:
    step_count += 1

    # ── Warmup ───────────────────────────────────────────────────────────────
    if step_count <= WARMUP_STEPS:
        set_motors(0.0, 0.0)
        continue

    # ── 1. Sense ─────────────────────────────────────────────────────────────
    rx, ry    = get_position()
    bearing   = get_bearing()
    dist      = distance_to_goal(rx, ry)
    goal_bear = angle_to_goal(rx, ry)
    herr      = normalize_angle(goal_bear - bearing)

    # FIX-2: Pass update_model=True ONLY during STATE_SEEK.
    # During avoidance the floor model is frozen to prevent obstacle-colour
    # contamination of the running floor baseline.
    update_floor = (current_state == STATE_SEEK)
    lm, cm, rm, left_blocked, center_blocked, right_blocked, \
        gray_u8, frame_bgr, obs_mask = process_camera_floor_model(
            floor_model, update_model=update_floor)

    # ── 2. Tick cooldown counters ─────────────────────────────────────────────
    if cooldown_counter > 0:
        cooldown_counter -= 1

    # FIX-5: Tick the stride interrupt cooldown independently
    if stride_interrupt_cooldown > 0:
        stride_interrupt_cooldown -= 1

    # Reset the one-frame interrupt display flag
    stride_just_interrupted = False

    # ── 3. Goal reached ───────────────────────────────────────────────────────
    if current_state != STATE_ARRIVED and dist < GOAL_TOLERANCE_M:
        current_state = STATE_ARRIVED
        set_motors(0.0, 0.0)
        print(f"[NAV] ARRIVED  pos=({rx:.3f},{ry:.3f})  "
              f"dist={dist:.4f} m  steps={step_count}")

    # ── 4. Avoidance state transitions ────────────────────────────────────────

    if current_state == STATE_AVOID_TURN:
        avoid_counter -= 1
        if avoid_counter <= 0:
            # Phase 1 complete → begin forward stride
            current_state = STATE_AVOID_DRIVE
            avoid_counter = STRIDE_DURATION
            print(f"[NAV] TURN→STRIDE  step={step_count}  "
                  f"pos=({rx:.2f},{ry:.2f})")

    elif current_state == STATE_AVOID_DRIVE:
        avoid_counter -= 1

        # ── FIX-3 / FIX-E: STRIDE INTERRUPT CHECK ────────────────────────────
        # Every step during the stride, check whether a new obstacle has
        # entered the center zone. If so, abort the stride immediately and
        # begin a new turn in the OPPOSITE direction (FIX-4).
        #
        # Guard: stride_interrupt_cooldown == 0 prevents rapid oscillation.
        # FIX-A ensures cooldown > turn_duration so the cooldown never expires
        # mid-turn (which was the root cause of the step-1 re-fire loop).
        if center_blocked and stride_interrupt_cooldown == 0:
            # FIX-4: New turn direction = opposite of the current avoidance
            # direction. This steers AWAY from the new obstacle, not back
            # into the one we were already avoiding.
            new_avoid_direction = -avoid_direction

            stride_just_interrupted = True   # FIX-6: trigger display indicator
            stride_interrupt_cooldown = STRIDE_INTERRUPT_COOLDOWN  # FIX-A

            # FIX-B: Escalate turn duration with each consecutive interrupt.
            # interrupt_consecutive is incremented here; it was reset to 0 on
            # the last clean stride completion (FIX-F).
            interrupt_consecutive += 1
            escalated_turn = min(
                STRIDE_INTERRUPT_TURN_DURATION
                + (interrupt_consecutive - 1) * INTERRUPT_ESCALATION_STEP,
                MAX_INTERRUPT_TURN_DURATION
            )

            # FIX-C / FIX-G: Stall detection.
            # Check whether the robot has moved meaningfully since the anchor
            # was set at avoidance entry. If not, this interrupt increments the
            # stall counter. When STALL_TRIGGER is reached, override to a full
            # TURN_DURATION escape turn regardless of the escalation schedule.
            stall_displaced = (math.hypot(rx - stall_anchor[0], ry - stall_anchor[1])
                               if stall_anchor else float('inf'))
            if stall_displaced < STALL_RADIUS:
                stall_count += 1
            else:
                stall_count = 0          # meaningful movement — reset stall count
                stall_anchor = (rx, ry)  # update anchor to new position

            if stall_count >= STALL_TRIGGER:
                # Robot is genuinely stuck against the wall — force a full
                # TURN_DURATION escape turn to gain angular clearance.
                escalated_turn = TURN_DURATION
                stall_count = 0
                print(f"[STALL] Forced escape at step={step_count}  "
                      f"pos=({rx:.2f},{ry:.2f})  "
                      f"displaced={stall_displaced:.3f} m  "
                      f"turn={escalated_turn} steps")
            else:
                print(f"[FIX] STRIDE INTERRUPTED at step={step_count}  "
                      f"pos=({rx:.2f},{ry:.2f})  "
                      f"new_dir={'LEFT' if new_avoid_direction == -1 else 'RIGHT'}  "
                      f"stride_remaining={avoid_counter} steps  "
                      f"consecutive={interrupt_consecutive}  "
                      f"turn={escalated_turn} steps")

            # Transition: re-enter turn phase with escalated duration
            current_state   = STATE_AVOID_TURN
            avoid_direction = new_avoid_direction
            avoid_counter   = escalated_turn
            # ── end FIX-3 / FIX-E ────────────────────────────────────────────

        elif avoid_counter <= 0:
            # Normal stride completion → return to goal-seeking
            # FIX-F: Reset consecutive interrupt counter — clean stride achieved
            interrupt_consecutive = 0
            stall_count           = 0
            current_state    = STATE_SEEK
            cooldown_counter = AVOID_COOLDOWN
            print(f"[NAV] STRIDE COMPLETE → SEEK  step={step_count}  "
                  f"pos=({rx:.2f},{ry:.2f})")

    # ── 5. Avoidance entry (from SEEK only) ───────────────────────────────────
    if current_state == STATE_SEEK and center_blocked and cooldown_counter == 0:
        current_state = STATE_AVOID_TURN
        avoid_counter = TURN_DURATION

        # Turn direction: reactive (open side) biased toward goal direction
        reactive_dir  = -1 if lm > rm else 1
        goal_bias_dir = -1 if herr > 0 else 1

        if reactive_dir == goal_bias_dir:
            avoid_direction = reactive_dir
        else:
            avoid_direction = (reactive_dir if abs(herr) > LARGE_HEADING_ERR
                               else goal_bias_dir)

        # FIX-F: Fresh avoidance from SEEK — reset escalation and stall state
        interrupt_consecutive = 0
        stall_count           = 0
        stall_anchor          = (rx, ry)   # FIX-G: anchor position at entry

        print(f"[OBS] Obstacle detected  step={step_count}  "
              f"pos=({rx:.2f},{ry:.2f})  "
              f"L/C/R blocked={left_blocked}/{center_blocked}/{right_blocked}  "
              f"turn={'LEFT' if avoid_direction == -1 else 'RIGHT'}  "
              f"floor_LAB={floor_model.mean.round(1)}")

    # ── 6. Motor commands ─────────────────────────────────────────────────────
    label = ""

    if current_state == STATE_ARRIVED:
        set_motors(0.0, 0.0)
        label = "ARRIVED"

    elif current_state == STATE_AVOID_TURN:
        if avoid_direction == -1:
            set_motors(-TURN_SPEED, TURN_SPEED)
            label = "AVOID: TURN LEFT"
        else:
            set_motors(TURN_SPEED, -TURN_SPEED)
            label = "AVOID: TURN RIGHT"

    elif current_state == STATE_AVOID_DRIVE:
        # Drive forward at full forward speed — obstacle detection now active
        # (FIX-3 handles any hit that occurs during this phase)
        set_motors(FORWARD_SPEED, FORWARD_SPEED)
        label = f"AVOID: STRIDE FWD ({avoid_counter})"

    elif current_state == STATE_SEEK:

        if abs(herr) > LARGE_HEADING_ERR:
            turn_cmd = max(-TURN_SPEED, min(TURN_SPEED, HEADING_KP * herr))
            set_motors(-turn_cmd, turn_cmd)
            label = f"ROTATE {'L' if herr > 0 else 'R'}"

        else:
            correction = max(-1.0, min(1.0, HEADING_KP * herr))

            # Soft lateral nudge for peripheral side obstacles
            if left_blocked and not center_blocked:
                correction = min(1.0, correction + 0.40)
            elif right_blocked and not center_blocked:
                correction = max(-1.0, correction - 0.40)

            # Decelerate near goal
            speed_scale = min(1.0, dist / SLOWDOWN_RADIUS)
            fwd = FORWARD_SPEED * max(MIN_SPEED_FRAC, speed_scale)

            delta = correction * (FORWARD_SPEED - STEER_SPEED)
            set_motors(fwd - delta, fwd + delta)

            label = ("FORWARD" if abs(herr) < HEADING_DEADBAND
                     else f"STEER {'L' if herr > 0 else 'R'}")

    # ── 7. Debug windows ─────────────────────────────────────────────────────
    if SHOW_DEBUG_WINDOWS and label:
        draw_debug(frame_bgr, gray_u8, obs_mask, lm, cm, rm,
                   label, dist, math.degrees(herr),
                   stride_interrupted=stride_just_interrupted)  # FIX-6
