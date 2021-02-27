// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int LEFT_MOTOR_PORT = 0;
    public static final int RIGHT_MOTOR_PORT = 1;

    // Quadrature Encoders on the ROMI - uses 2 ports per encoder
    // Left Side with Ports 4 and 5
    public static final int LEFT_ENCODER_A = 4;
    public static final int LEFT_ENCODER_B = 5;
    // Right Side with Ports 6 and 7
    public static final int RIGHT_ENCODER_A = 6;
    public static final int RIGHT_ENCODER_B = 7;

    // Additional parameters - not specific PORTS related
    // Wheel diameter is 2.83"
    // public static final double WHEEL_DIA = 2.83;
    public static final double WHEEL_DIA = 2.76;
    // ROMI's encoder is 1440 pulse per revolution 
    //  => resolution = 1440 pulses/360 degree or 4 pulse/degree of rotations) or 0.25 degree/pulse
    // public static final double PULSES_PER_REVOLUTION = 1440.0;
    public static final double PULSES_PER_REVOLUTION = 1437.0;
    // convert to inches per revolution -> (Dia * Pi)/Pulse_Per_Rev
    public static final double INCHES_PER_PULSE = Math.PI * WHEEL_DIA / PULSES_PER_REVOLUTION;
    //
    // Width of Robot = Wheel_Track
    public static final double WHEEL_TRACK = 5.25;
    public static final double INCHES_PER_TURN_DEGREE = Math.PI * WHEEL_TRACK / 360.0;

    // 
    public static final int ControllerPort = 1;
}
