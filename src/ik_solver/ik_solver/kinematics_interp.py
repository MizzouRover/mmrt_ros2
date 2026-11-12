# ---------------------------------------------------------------------------
# Inverse kinematics for the rover arm
# Made by spencer with syntactical help from claude
#
# Frame: x = forward, y = left/right (along the slide), z = up.
# Units: meters and radians.
#
# Inputs (the target you want the claw to reach)
#   x, y, z     target claw tip position, measured from the base frame origin
#   pitch       angle the claw points, measured from horizontal (+ = up)
#   roll        rotation of the claw about its own pointing axis
#   elbow_up    True = elbow bends above the shoulder-wrist line
#
# Geometry (ArmGeometry, fixed for this arm)
#   l1          length from shoulder pivot to elbow pivot
#   l2          length from elbow pivot to wrist pitch pivot
#   l3          length from wrist pitch pivot to claw tip
#   shoulder_x  shoulder pivot position relative to the base frame
#   shoulder_z
#   slide_min   linear slide travel limits
#   slide_max
#
# Intermediate Values (inside solve_planar)
#   px, pz      target claw tip position, measured from the shoulder
#   wx, wz      wrist pitch pivot position, measured from the shoulder
#   r_sq        squared distance from shoulder to wrist (wx^2 + wz^2)
#   cos_elbow   cosine of the elbow bend, from the law of cosines
#
# Outputs (JointSolution)
#   slide        linear slide position
#   shoulder     angle of link 1 above horizontal
#   elbow        angle of link 2 relative to link 1 (0 = straight)
#   wrist_pitch  angle of link 3 relative to link 2
#   wrist_roll   claw rotation about its pointing axis
#
# Note: shoulder + elbow + wrist_pitch = pitch
# ---------------------------------------------------------------------------

from curses import window
from dataclasses import dataclass
import math
import csv
import sys
import tkinter as tk
import time

# There are 3 main parts to the solution.
# The linear slide. (y axis)
# The shoulder/elbow/wrist_pitch angles (x/z plane)
# The wrist_roll angle. 
# With the way the arm is designed, these three parts are independent of each other, so they can be solved separately.
# So to get the solution, we just solve each part and combine the results.

class UnreachableError(ValueError):
    """Target is outside the arm's reach."""

@dataclass(frozen=True)
class ArmGeometry:
    l1: float
    l2: float
    l3: float
    shoulder_x: float
    shoulder_z: float
    slide_min: float
    slide_max: float


@dataclass(frozen=True)
class JointSolution:
    slide: float
    shoulder: float
    elbow: float
    wrist_pitch: float
    wrist_roll: float
 
    def as_list(self) -> list[float]:
        return [self.slide, self.shoulder, self.elbow,
                self.wrist_pitch, self.wrist_roll]

@dataclass(frozen=True)
class ClawTipPosition:
    x: float
    y: float
    z: float


def wrap_angle(angle: float) -> float:
    """Wrap angle to be within [-pi, pi]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


# Linear Slide Calculation

def solve_slide(y: float, geo: ArmGeometry) -> float:
    if not geo.slide_min <= y <= geo.slide_max:
        raise UnreachableError(f'y={y:.3f} outside slide range [{geo.slide_min}, {geo.slide_max}]')
    return y


# Shoulder/Elbow/Wrist Pitch Calculation

def solve_planar(x: float, z: float, pitch: float, geo: ArmGeometry, elbow_up: bool = True) -> tuple[float, float, float]:
    px = x - geo.shoulder_x # distance from shoulder to tip of the tool
    pz = z - geo.shoulder_z

    wx = px - geo.l3 * math.cos(pitch) # distance from shoulder to wrist 
    wz = pz - geo.l3 * math.sin(pitch)
    r_sq = wx * wx + wz * wz # distance formula

    # law of cosines for the elbow.
    cos_elbow = (r_sq - geo.l1 ** 2 - geo.l2 ** 2) / (2.0 * geo.l1 * geo.l2)
    if abs(cos_elbow) > 1.0 + 1e-9: # for rounding error Purposes
        raise UnreachableError(f'wrist point ({wx:.3f}, {wz:.3f}) is out of reach of links 1+2')
    cos_elbow = max(-1.0, min(1.0, cos_elbow)) # clamp float noise
 
    elbow = math.acos(cos_elbow)

    # two mirror-image solutions: bend one way (+elbow) or the other (-elbow)
    shoulder_pos = math.atan2(wz, wx) - math.atan2(
        geo.l2 * math.sin(elbow), geo.l1 + geo.l2 * math.cos(elbow)) # use atan2 for quadrant awareness
    shoulder_neg = math.atan2(wz, wx) - math.atan2(
        geo.l2 * math.sin(-elbow), geo.l1 + geo.l2 * math.cos(-elbow))

    # the elbow joint sits l1 * sin(shoulder) above the shoulder,
    # so keep whichever solution puts it higher (elbow_up) or lower (elbow down) in world z
    neg_is_higher = math.sin(shoulder_neg) > math.sin(shoulder_pos)
    if elbow_up == neg_is_higher:
        shoulder = shoulder_neg
        elbow = -elbow
    else:
        shoulder = shoulder_pos

    # whatever pitch is left over goes to the wrist.
    wrist_pitch = pitch - shoulder - elbow

    return wrap_angle(shoulder), wrap_angle(elbow), wrap_angle(wrist_pitch)


# Wrist Roll Calculation

def solve_wrist_roll(roll: float) -> float:
    return wrap_angle(roll)

# Full Solve

def solve(x: float, y: float, z: float, pitch: float, roll: float, geo: ArmGeometry, elbow_up: bool = True) -> JointSolution:
    slide = solve_slide(y, geo)
    shoulder, elbow, wrist_pitch = solve_planar(x, z, pitch, geo, elbow_up)
    wrist_roll = solve_wrist_roll(roll)
    return JointSolution(slide, shoulder, elbow, wrist_pitch, wrist_roll)

# Outputs (JointSolution)
#   slide        linear slide position
#   shoulder     angle of link 1 above horizontal
#   elbow        angle of link 2 relative to link 1 (0 = straight)
#   wrist_pitch  angle of link 3 relative to link 2
#   wrist_roll   claw rotation about its pointing axis

#class ClawTipPosition:
#    x: float
#    y: float
#    z: float

def fk_solver(joint_solution: JointSolution, geo: ArmGeometry) -> ClawTipPosition:
    slide = joint_solution.slide
    shoulder = joint_solution.shoulder
    elbow = joint_solution.elbow
    wrist_pitch = joint_solution.wrist_pitch

    l1_x = geo.l1 * math.cos(shoulder)
    l1_z = geo.l1 * math.sin(shoulder)

    l2_x = geo.l2 * math.cos(shoulder + elbow)
    l2_z = geo.l2 * math.sin(shoulder + elbow)

    l3_x = geo.l3 * math.cos(shoulder + elbow + wrist_pitch)
    l3_z = geo.l3 * math.sin(shoulder + elbow + wrist_pitch)

    lx = l1_x + l2_x + l3_x
    lz = l1_z + l2_z + l3_z

    x = geo.shoulder_x + lx
    y = slide
    z = geo.shoulder_z + lz

    return ClawTipPosition(x=x, y=y, z=z)
    
def main():
    # Example usage
    geo = ArmGeometry(
            l1=2.0, l2=2.0, l3=0.5,
            shoulder_x=0.0, shoulder_z=0.0,
            slide_min=-0.1, slide_max=0.1
        )

    # Each trajectory moves the target in a straight line from start to end.
    # (name, start (x, y, z, pitch_deg, roll_deg), end (x, y, z, pitch_deg, roll_deg), steps, elbow_up)
    trajectories = [
        ("reach out level",        (2.5, 0.0, -0.5,   0,   0), (4.3, 0.0, -0.5,   0,   0), 40, True),
        ("reach out, elbow down",  (2.5, 0.0,  0.5,   0,   0), (4.3, 0.0,  0.5,   0,   0), 40, False),
        ("sweep high to low",      (3.5, 0.0,  1.0,   0,   0), (3.5, 0.0, -1.5,   0,   0), 40, True),
        ("pitch sweep",            (3.0, 0.0,  0.0, -90,   0), (3.0, 0.0,  0.0,  45,   0), 40, True),
        ("diagonal pull in",       (4.0, 0.0, -1.0, -45,   0), (2.5, 0.0, -0.5,   0,   0), 40, True),
        ("slide across",           (3.0, -0.1, 0.0,   0,   0), (3.0, 0.1,  0.0,   0,   0), 40, True),
        ("roll past 180",          (3.0, 0.0,  0.0,   0,   0), (3.0, 0.0,  0.0,   0, 270), 40, True),
    ]
    max_step_deg = 10.0  # any joint moving more than this in one step counts as a jump

    frames = []  # (name, step, target_x, target_z, solution, jump) for every reachable step

    with open("diff_results.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["trajectory", "step", 
                         "slide", "shoulder_deg", "elbow_deg",
                         "wrist_pitch_deg", "wrist_roll_deg",
                         "dshoulder_deg", "delbow_deg",
                         "dwrist_pitch_deg", "dwrist_roll_deg"
                         ])

        for name, start, end, steps, elbow_up in trajectories:
            prev = None  # joint angles from the previous step of this trajectory
            for k in range(steps + 1):
                t = k / steps
                x, y, z, pitch_deg, roll_deg = [a + (b - a) * t for a, b in zip(start, end)]
                try:
                    solution = solve(x, y, z, math.radians(pitch_deg), math.radians(roll_deg), geo, elbow_up)
                    tip = fk_solver(solution, geo)

                    # calculate angle diffs each step
                    angles = [solution.shoulder, solution.elbow, solution.wrist_pitch, solution.wrist_roll]
                    diff_angles = angles
                    if prev is None:
                        diff_angles = [0, 0, 0, 0];
                    else:
                        diff_angles = [solution.shoulder - prev[0],
                                       solution.elbow - prev[1],
                                       solution.wrist_pitch - prev[2],
                                       solution.wrist_roll - prev[3]];

                    prev = angles
                    jump = False;

                    writer.writerow([name, k, 
                                     f"{solution.slide:.3f}",
                                     f"{math.degrees(solution.shoulder):.3f}",
                                     f"{math.degrees(solution.elbow):.3f}",
                                     f"{math.degrees(solution.wrist_pitch):.3f}",
                                     f"{math.degrees(solution.wrist_roll):.3f}",
                                     f"{math.degrees(diff_angles[0]):.3f}",
                                     f"{math.degrees(diff_angles[1]):.3f}",
                                     f"{math.degrees(diff_angles[2]):.3f}",
                                     f"{math.degrees(diff_angles[3]):.3f}",
                                     ])
                    frames.append((name, k, x, z, solution, jump))
                except UnreachableError as e:
                    prev = None  # don't compare across a gap
                    writer.writerow([name, k, 
                                     "", "", "", "", "", "", "", "", "", "", "",
                                     str(e)])

    print("Wrote diff_results.csv")

    if not frames:
        print("No reachable targets to visualize")
        return

    # First pass: compute every joint position so the view can be fit to the motion
    arm_frames = []  # (name, step, target_x, target_z, [shoulder, elbow, wrist, tip] as (x, z), jump)
    xs = []
    zs = []
    for name, k, x, z, sol, jump in frames:
        # same chain as fk_solver, but keeping each joint position
        sx = geo.shoulder_x
        sz = geo.shoulder_z
        ex = sx + geo.l1 * math.cos(sol.shoulder)
        ez = sz + geo.l1 * math.sin(sol.shoulder)
        wx = ex + geo.l2 * math.cos(sol.shoulder + sol.elbow)
        wz = ez + geo.l2 * math.sin(sol.shoulder + sol.elbow)
        tx = wx + geo.l3 * math.cos(sol.shoulder + sol.elbow + sol.wrist_pitch)
        tz = wz + geo.l3 * math.sin(sol.shoulder + sol.elbow + sol.wrist_pitch)

        arm_frames.append((name, k, x, z, [(sx, sz), (ex, ez), (wx, wz), (tx, tz)], jump))
        xs += [sx, ex, wx, tx, x]
        zs += [sz, ez, wz, tz, z]

    # Zoom: fit the view to everything the arm and targets touch
    zoom = 1.0  # raise above 1 to zoom in further on the center, lower to zoom out
    size = 600
    mid = size / 2
    span = max(max(xs) - min(xs), max(zs) - min(zs)) or 1.0
    scale = zoom * (size - 40) / span  # pixels per unit, with a 20px margin
    center_x = (min(xs) + max(xs)) / 2
    center_z = (min(zs) + max(zs)) / 2

    # Visualizer: side view of the arm in the x/z plane
    root = tk.Tk()
    root.title("Arm IK visualizer")
    canvas = tk.Canvas(root, width=size, height=size, bg="white")
    canvas.pack()

    # axes through the shoulder
    shoulder_cx = mid + (geo.shoulder_x - center_x) * scale
    shoulder_cy = mid - (geo.shoulder_z - center_z) * scale
    canvas.create_line(0, shoulder_cy, size, shoulder_cy, fill="#ddd")
    canvas.create_line(shoulder_cx, 0, shoulder_cx, size, fill="#ddd")

    arm = canvas.create_line(0, 0, 0, 0, width=4, fill="steelblue")
    joints = [canvas.create_oval(0, 0, 0, 0, fill="black") for _ in range(4)]
    target = canvas.create_text(0, 0, text="×", fill="red", font=("Arial", 18, "bold"))
    label = canvas.create_text(10, 10, anchor="nw", text="", font=("Arial", 12))

    for name, k, x, z, joint_positions, jump in arm_frames:
        # convert to canvas pixels (canvas y grows downward, so z is flipped)
        points = [(mid + (px - center_x) * scale, mid - (pz - center_z) * scale)
                  for px, pz in joint_positions]

        try:
            if k == 0:
                canvas.delete("trail")  # new trajectory, clear the old elbow trail

            # elbow trail: a sudden inversion shows up as a gap in the dotted path
            ecx, ecy = points[1]
            canvas.create_oval(ecx - 2, ecy - 2, ecx + 2, ecy + 2, fill="orange", outline="", tags="trail")

            canvas.coords(arm, *[c for p in points for c in p])
            canvas.itemconfig(arm, fill="red" if jump else "steelblue")
            for joint, (cx, cy) in zip(joints, points):
                canvas.coords(joint, cx - 5, cy - 5, cx + 5, cy + 5)
            canvas.tag_raise(arm)
            for joint in joints:
                canvas.tag_raise(joint)
            canvas.coords(target, mid + (x - center_x) * scale,
                          mid - (z - center_z) * scale)
            canvas.itemconfig(label, text=f"{name}  step {k}" + ("   JUMP" if jump else ""))
            root.update()
        except tk.TclError:  # window was closed mid-animation
            break

        time.sleep(1.0 if jump or k == 0 else 0.05)  # linger on new trajectories and jumps
    else:
        root.mainloop()  # keep the window open on the last frame

    
if __name__ == "__main__":
    main()
