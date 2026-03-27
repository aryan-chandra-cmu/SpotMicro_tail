# Written by team Tailenders (Atharva Sunder, Kausik Kolluri, Jash Lapsiwala, Aryan Chandra, Raymond Cao)
# with assistance from Claude Opus 4.6
# Date: 19th March 2026

"""
Servo class for commanding individual servos via a PCA9685 driver.

Each Servo instance is bound to a specific channel on a PCA9685 board.
It handles angle-to-pulse-width conversion and sends the final PWM command.
"""


class Servo:
    """Controls a single servo on a PCA9685 PWM driver."""

    def __init__(self, pca, channel, min_pulse, max_pulse, min_angle, max_angle):
        """
        Initialize a Servo instance.

        Parameters
        ----------
        pca : adafruit_pca9685.PCA9685
            The PCA9685 driver object (created in main.py).
        channel : int
            PWM channel number on the PCA9685 (0-15).
        min_pulse : int
            Pulse width in microseconds corresponding to min_angle.
        max_pulse : int
            Pulse width in microseconds corresponding to max_angle.
        min_angle : float
            Minimum servo angle in degrees.
        max_angle : float
            Maximum servo angle in degrees.
        """
        self.pca = pca
        self.channel = channel
        self.min_pulse = min_pulse
        self.max_pulse = max_pulse
        self.min_angle = min_angle
        self.max_angle = max_angle

    def angle_to_pulse_width(self, angle):
        """
        Convert a desired servo angle (degrees) to a pulse width (microseconds).

        Performs linear interpolation between (min_angle, min_pulse) and
        (max_angle, max_pulse). Clamps the angle to [min_angle, max_angle].

        Parameters
        ----------
        angle : float
            Desired servo angle in degrees.

        Returns
        -------
        int
            Pulse width in microseconds.
        """
        clamped = max(self.min_angle, min(angle, self.max_angle))
        proportion = (clamped - self.min_angle) / (self.max_angle - self.min_angle)
        pulse = self.min_pulse + proportion * (self.max_pulse - self.min_pulse)
        return int(pulse)

    def command(self, angle):
        """
        Command the servo to a desired angle.

        Converts the angle to a pulse width and writes the duty cycle
        to the PCA9685 channel.

        Parameters
        ----------
        angle : float
            Desired servo angle in degrees.
        """
        pulse_us = self.angle_to_pulse_width(angle)

        # PCA9685 runs at 50 Hz → period = 20 000 µs → 16-bit range (0–65535)
        # duty_cycle = pulse_us / 20000 * 65535
        duty_cycle = int(pulse_us / 20000.0 * 65535)
        duty_cycle = max(0, min(duty_cycle, 65535))

        self.pca.channels[self.channel].duty_cycle = duty_cycle
