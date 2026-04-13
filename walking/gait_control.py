import numpy as np
import matplotlib.pyplot as plt
from spot_config import WALKING_PARAMS

class Trot:
    def __init__(self):
        # We don't need dt for the parametric function itself, 
        # but we use it for plotting and simulating steps.
        self.step_duration = WALKING_PARAMS["step_duration"]
        self.stance_duration = 0.5 * self.step_duration
        self.swing_duration = 0.5 * self.step_duration

        self.legs = ["left_front", "left_rear", "right_front", "right_rear"]

    def get_foot_position(self, leg_name, t, current_forward_speed):
        """
        Dynamically computes the (x, y, z) foot position for a given leg at time t,
        based on the provided forward speed.
        
        If current_forward_speed is 0, the dog trots in place (step_length = 0).
        """
        params = WALKING_PARAMS[leg_name]
        step_height = params["step_height"]
        ground_z = params["ground_z"]
        y_offset = params["y_offset"]
        
        # Step length is dynamically computed based on speed and stance duration.
        # This determines how far the foot must travel backward during stance.
        dynamic_step_length = current_forward_speed * self.stance_duration

        # Phase offsets for a diagonal trot
        phase_offsets = {
            "left_front": 0.0,
            "right_rear": 0.0,
            "right_front": 0.5 * self.step_duration,
            "left_rear": 0.5 * self.step_duration,
        }

        leg_time = (t + phase_offsets[leg_name]) % self.step_duration

        if leg_time < self.swing_duration:
            # =========================
            # SWING PHASE
            # =========================
            # Time progress through the swing phase (0.0 to 1.0)
            s = leg_time / self.swing_duration
            
            # x goes from -L/2 to +L/2 using a cosine profile for smooth acceleration
            x = -(dynamic_step_length / 2.0) * np.cos(s * np.pi)
            
            # z follows a smooth sine wave arc
            z = ground_z - step_height * np.sin(s * np.pi)
            
        else:
            # =========================
            # STANCE PHASE
            # =========================
            # Time progress through the stance phase (0.0 to 1.0)
            s = (leg_time - self.swing_duration) / self.stance_duration
            
            # x goes linearly from +L/2 down to -L/2
            x = (dynamic_step_length / 2.0) - s * dynamic_step_length
            
            z = ground_z

        y = y_offset

        return float(x), float(y), float(z)

if __name__ == '__main__':
    trot = Trot()

    # Ramping up speed demonstration
    speeds_to_test = [0.0, 0.05, 0.15]
    
    print("Testing get_foot_position with different forward speeds...")
    t = 0.0
    for speed in speeds_to_test:
        print(f"\n--- Forward Speed: {speed} m/s ---")
        for leg_name in ["left_front", "right_front", "left_rear", "right_rear"]:
            pos = trot.get_foot_position(leg_name, t, current_forward_speed=speed)
            print(f"{leg_name} at t=0.0: (x={pos[0]:.4f}, y={pos[1]:.4f}, z={pos[2]:.4f})")
            
    # Visualize the trajectory for a steady speed
    current_forward_speed = 0.0
    leg_name = "left_front"
    dt = 0.02

    times = np.arange(0, trot.step_duration, dt)
    
    x_vals = []
    y_vals = []
    z_vals = []
    
    for t in times:
        x, y, z = trot.get_foot_position(leg_name, t, current_forward_speed)
        x_vals.append(x)
        y_vals.append(y)
        z_vals.append(z)
        
    x_vals = np.array(x_vals)
    y_vals = np.array(y_vals)
    z_vals = np.array(z_vals)
    
    vx_vals = np.gradient(x_vals, dt)
    vz_vals = np.gradient(z_vals, dt)

    # Plot X and Z positions
    plt.figure(figsize=(10, 4))
    plt.plot(times, x_vals, marker='o')
    plt.xlabel("Time (s)")
    plt.ylabel("x position")
    plt.title(f"Dynamic X trajectory vs Time ({leg_name}, V_fwd={current_forward_speed})")
    plt.grid(True)
    plt.show()

    plt.figure(figsize=(10, 4))
    plt.plot(times, z_vals, marker='o')
    plt.xlabel("Time (s)")
    plt.ylabel("z position")
    plt.title(f"Dynamic Z trajectory vs Time ({leg_name}, V_fwd={current_forward_speed})")
    plt.grid(True)
    plt.show()

    # Plot velocities
    plt.figure(figsize=(10, 4))
    plt.plot(times, vx_vals, marker='o')
    plt.xlabel("Time (s)")
    plt.ylabel("vx")
    plt.title(f"X Velocity vs Time ({leg_name})")
    plt.grid(True)
    plt.show()

    plt.figure(figsize=(10, 4))
    plt.plot(times, vz_vals, marker='o')
    plt.xlabel("Time (s)")
    plt.ylabel("vz")
    plt.title(f"Z Velocity vs Time ({leg_name})")
    plt.grid(True)
    plt.show()

    # Plot X-Z path
    plt.figure(figsize=(6, 6))
    plt.plot(x_vals, z_vals, marker='o')
    plt.xlabel("x position")
    plt.ylabel("z position")
    plt.title(f"Foot trajectory in X-Z plane ({leg_name})")
    plt.grid(True)
    plt.axis("equal")
    plt.show()