import math
import numpy as np
import matplotlib.pyplot as plt

def inverse_kinematics(x, y, z):
    """
    Inverse kinematics for a 3-DOF leg.

    Axes:
        x : forward / out of plotting plane
        y : left
        z : down

    Returns
    -------
    knee_deg, hip_deg, shoulder_deg
    """

    # Link lengths in mm
    upper_leg = 120.0
    lower_leg = 120.0
    shoulder_offset = 40.0

    # Quantity used to reduce 3D geometry to the hip-knee plane
    inside = y**2 + z**2 - shoulder_offset**2

    y1 = math.sqrt(inside)

    # Distance from hip joint to foot in the hip-knee plane
    distance = math.sqrt(x**2 + y1**2)

    # Reachability check
    if distance > upper_leg + lower_leg or distance < abs(upper_leg - lower_leg):
        raise ValueError("Target is outside the reachable workspace of the hip-knee chain.")

    # Clamp acos argument for numerical safety
    cos_knee = (distance**2 - upper_leg**2 - lower_leg**2) / (-2.0 * upper_leg * lower_leg)
    cos_knee = max(-1.0, min(1.0, cos_knee))

    knee = math.acos(cos_knee)

    # Obtain hip angle
    hip = math.asin((lower_leg * math.sin(knee)) / distance) + math.atan(abs(x)/y1)

    # Obtain shoulder angle
    shoulder = math.atan2(y1, shoulder_offset) + math.atan2(-y, z)

    # Convert to degrees
    knee_deg = math.degrees(knee)
    hip_deg = math.degrees(hip)
    shoulder_deg = math.degrees(shoulder)

    return knee_deg, hip_deg, shoulder_deg

def map_to_servo_angles(knee_deg, hip_deg, shoulder_deg):
    """
    Convert IK joint angles to servo-coordinate angles.

    Parameters
    ----------
    knee_deg, hip_deg, shoulder_deg : float
        Joint angles from inverse_kinematics() in degrees.

    Returns
    -------
    theta_servo_knee, theta_servo_hip, theta_servo_shoulder : float
        Corresponding servo angles in degrees.
    """
    theta_servo_knee = knee_deg
    theta_servo_hip = hip_deg + 90.0
    theta_servo_shoulder = 180.0 - shoulder_deg
    return theta_servo_knee, theta_servo_hip, theta_servo_shoulder


def compute_geometry(x, y, z):
    """
    Compute all points needed for plotting.

    Returns a dictionary with:
    - shoulder origin in front view
    - lateral shoulder joint in front view
    - foot in front view
    - hip-knee-plane points for the second plot
    - IK angles
    """

    upper_leg = 120.0
    lower_leg = 120.0
    shoulder_offset = 40.0

    knee_deg, hip_deg, shoulder_deg = inverse_kinematics(x, y, z)

    knee = math.radians(knee_deg)
    hip = math.radians(hip_deg)
    shoulder = math.radians(shoulder_deg)

    # This is the reduced in-plane distance from shoulder-side geometry
    y1 = math.sqrt(y**2 + z**2 - shoulder_offset**2)

    # ------------------------------------------------------------------
    # Plot 1: Front view (y-z plane)
    # Shoulder origin is at (0,0).
    # The offset link extends at shoulder_deg from the +z (down) axis.
    # The foot is at distance y1 from the lateral joint, perpendicular
    # to the offset link.
    # ------------------------------------------------------------------
    shoulder_origin_front = np.array([0.0, 0.0])               # (y, z)
    shoulder_lateral_front = np.array([
        -shoulder_offset * math.sin(shoulder),
        shoulder_offset * math.cos(shoulder)
    ])
    foot_front = np.array([
        shoulder_lateral_front[0] + y1 * math.cos(shoulder),
        shoulder_lateral_front[1] + y1 * math.sin(shoulder)
    ])

    # ------------------------------------------------------------------
    # Plot 2: Hip-knee plane
    #
    # Hip is at the origin.  Vertical axis points downward
    #
    # hip_deg (from inverse_kinematics):
    #   angle between the vertical axis and the thigh link.
    #
    # knee_deg (from inverse_kinematics):
    #   angle between the thigh link and the shank link.
    #
    # Knee point (end of thigh):
    #   x_k = L1 * sin(hip)
    #   z_k = L1 * cos(hip)
    #
    # Foot point (end of shank):
    #   The absolute angle of the shank from vertical is (hip - knee),
    #   because the knee folds the shank back toward vertical.
    #   x_f = x_k + L2 * sin(hip - knee)
    #   z_f = z_k + L2 * cos(hip - knee)
    # ------------------------------------------------------------------

    hip_point_plane = np.array([0.0, 0.0])

    knee_point_plane = np.array([
        upper_leg * math.sin(hip),
        upper_leg * math.cos(hip)
    ])

    shank_abs_angle = hip + knee - math.pi
    print(shank_abs_angle)
    foot_point_plane = np.array([
        knee_point_plane[0] + lower_leg * math.sin(shank_abs_angle),
        knee_point_plane[1] + lower_leg * math.cos(shank_abs_angle)
    ])

    return {
        "angles_deg": {
            "shoulder": shoulder_deg,
            "hip": hip_deg,
            "knee": knee_deg
        },
        "front_view": {
            "shoulder_origin": shoulder_origin_front,
            "shoulder_lateral": shoulder_lateral_front,
            "foot": foot_front
        },
        "leg_plane": {
            "hip": hip_point_plane,
            "knee": knee_point_plane,
            "foot": foot_point_plane
        }
    }


def plot_leg_ik(x, y, z):
    """
    Make the two requested 2D plots for a given target foot position.
    """

    geom = compute_geometry(x, y, z)

    shoulder_deg = geom["angles_deg"]["shoulder"]
    hip_deg = geom["angles_deg"]["hip"]
    knee_deg = geom["angles_deg"]["knee"]

    front = geom["front_view"]
    plane = geom["leg_plane"]

    # Compute servo angles
    servo_knee, servo_hip, servo_shoulder = map_to_servo_angles(
        knee_deg, hip_deg, shoulder_deg
    )

    fig, axes = plt.subplots(1, 2, figsize=(12, 6))

    # ==============================================================
    # Plot 1: Front view in y-z plane
    # ==============================================================
    ax = axes[0]

    p0 = front["shoulder_origin"]
    p1 = front["shoulder_lateral"]
    pf = front["foot"]

    # Draw shoulder offset link
    ax.plot([p0[0], p1[0]], [p0[1], p1[1]], 'o-', linewidth=3, label='Shoulder offset link')

    # Draw line from lateral shoulder joint to foot projection
    ax.plot([p1[0], pf[0]], [p1[1], pf[1]], 'o--', linewidth=2, label='Foot line in y-z plane')

    # Mark points
    ax.text(p0[0], p0[1], '  Shoulder origin')
    ax.text(p1[0], p1[1], '  Shoulder joint')
    ax.text(pf[0], pf[1], '  Foot')

    ax.set_title(
        f"Front View (y-z plane)\n"
        f"Shoulder = {shoulder_deg:.2f} deg  |  Servo Shoulder = {servo_shoulder:.2f} deg"
    )
    ax.set_xlabel("y (left +)")
    ax.set_ylabel("z (down +)")
    ax.axis('equal')
    ax.grid(True)
    ax.legend()

    # Make +z point downward and +y point left on screen
    ax.invert_yaxis()
    ax.invert_xaxis()

    # ==============================================================
    # Plot 2: Hip-knee plane
    # ==============================================================
    ax = axes[1]

    ph = plane["hip"]
    pk = plane["knee"]
    pfp = plane["foot"]

    # Upper leg
    ax.plot([ph[0], pk[0]], [ph[1], pk[1]], 'o-', linewidth=3, label='Upper leg')

    # Lower leg
    ax.plot([pk[0], pfp[0]], [pk[1], pfp[1]], 'o-', linewidth=3, label='Lower leg')

    # Hip-to-foot line for reference
    ax.plot([ph[0], pfp[0]], [ph[1], pfp[1]], '--', linewidth=1.5, label='Hip-foot line')

    ax.text(ph[0], ph[1], '  Hip')
    ax.text(pk[0], pk[1], '  Knee')
    ax.text(pfp[0], pfp[1], '  Foot')

    ax.set_title(
        f"Hip-Knee Plane\n"
        f"Hip = {hip_deg:.2f} deg, Knee = {knee_deg:.2f} deg\n"
        f"Servo Hip = {servo_hip:.2f} deg, Servo Knee = {servo_knee:.2f} deg"
    )
    ax.set_xlabel("x")
    ax.set_ylabel("z")
    ax.axis('equal')
    ax.grid(True)
    ax.legend()

    # Make vertical axis increase downward
    ax.invert_yaxis()

    plt.tight_layout()
    plt.show()

    print(f"Input foot position: x={x:.2f} mm, y={y:.2f} mm, z={z:.2f} mm")
    print(f"Shoulder angle: {shoulder_deg:.3f} deg")
    print(f"Hip angle     : {hip_deg:.3f} deg")
    print(f"Knee angle    : {knee_deg:.3f} deg")
    print(f"\nServo Angles:")
    print(f"Servo Shoulder: {servo_shoulder:.3f} deg")
    print(f"Servo Hip     : {servo_hip:.3f} deg")
    print(f"Servo Knee    : {servo_knee:.3f} deg")


# ------------------------------------------------------------------
# Example usage
# ------------------------------------------------------------------
if __name__ == "__main__":
    # Example target
    x = 0.0
    y = -40.0
    z = 240.0

    plot_leg_ik(x, y, z)