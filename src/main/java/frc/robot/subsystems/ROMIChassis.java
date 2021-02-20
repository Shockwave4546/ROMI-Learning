// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ROMIChassis extends SubsystemBase {
  //
  // Defining or making references to the "HW" that are part of the Romi Chassis
  PWMVictorSPX leftMotor, rightMotor;
  Encoder leftEncoder, rightEncoder;
  DifferentialDrive diffDrive;

  /** Creates a new ROMIChassis. */
  public ROMIChassis() {
    // Instantiated or make it real the Left and Right Motor
    leftMotor = new PWMVictorSPX(Constants.LEFT_MOTOR_PORT);
    rightMotor = new PWMVictorSPX(Constants.RIGHT_MOTOR_PORT);

    // same for the Differential Drive System with just the Left and Right side motors
    diffDrive = new DifferentialDrive(leftMotor, rightMotor);
    // ROMI does not need the Right Side Inverted - see documentations
    diffDrive.setRightSideInverted(true);
    // ROMI also has a false safety feature that we should disabled
    diffDrive.setSafetyEnabled(false);

    // Encoders being the last
    leftEncoder = new Encoder(Constants.LEFT_ENCODER_A, Constants.LEFT_ENCODER_B);
    rightEncoder = new Encoder(Constants.RIGHT_ENCODER_A, Constants.RIGHT_ENCODER_B);
  
    // Set the resolution for both the left and right encoders
    leftEncoder.setDistancePerPulse(Constants.INCHES_PER_PULSE);
    rightEncoder.setDistancePerPulse(Constants.INCHES_PER_PULSE);

    // add these to the dashboard
    addChild("leftEncoder", leftEncoder);
    addChild("rightEncoder", rightEncoder);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Create the reset command to reset the left and right encoders
  public void reset() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  // Create the Drive Command
  public void driveForward(double speed, double direction) {
    // Use the differentialDrive with Speed and Direction as-is
    diffDrive.arcadeDrive(speed, direction);
    // diffDrive.tankDrive(0.97*speed, speed);
  }

  // Create the STOP command using the drive() function
  public void stop() {
    // Full Stop = drive at 0 speed and 0 direction
    driveForward(0, 0);
  }

  public double getLeftDistance() {
    // for now, we going to return the Left Encoder as if it's the same as the Right Encoder
    return leftEncoder.getDistance();
  }

  public double getRightDistance() {
    return rightEncoder.getDistance();
  }

  public double getDistance() {
    return (getLeftDistance() + getRightDistance())/2;
  }
}
