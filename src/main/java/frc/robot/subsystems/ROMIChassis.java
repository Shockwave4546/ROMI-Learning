// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sensors.RomiGyro;

public class ROMIChassis extends SubsystemBase {
  //
  // Defining or making references to the "HW" that are part of the Romi Chassis
  PWMVictorSPX leftMotor, rightMotor;
  Encoder leftEncoder, rightEncoder;
  DifferentialDrive diffDrive;
  RomiGyro gyro;
  XboxController xboxControl;

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

    // Instantiate the Gyro too
    gyro = new RomiGyro();
    xboxControl = new XboxController(Constants.ControllerPort);

    // add these to the dashboard
    addChild("leftEncoder", leftEncoder);
    addChild("rightEncoder", rightEncoder);
    // addChild("gyro", gyro);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Create the reset command to reset the left and right encoders and also the gyro too
  public void reset() {
    leftEncoder.reset();
    rightEncoder.reset();
    gyro.reset();
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

  public double getLeftCount() {
    return leftEncoder.get();
  }

  public double getRightCount() {
    return rightEncoder.get();
  }

  public double getCount() {
    return (leftEncoder.get() + rightEncoder.get())/2;
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

  public double getHeading() {
    // Use the ROMIGyro's getAngle instead of getAngleZ() directly
    return gyro.getAngle();
  }

  public void Turn(double speed, double direction) {
    diffDrive.arcadeDrive(speed, direction);
  }

  public void DBC(XboxController xboxControl) {
    // This gets it from Axis(4) and Axis(5) which is the right stick
    // diffDrive.arcadeDrive(xboxControl.getX(), xboxControl.getY());
    // 0 and 1 is the left stick - use getRawAxis() to get the value
    //
    // remember, axis on the controller are inverted - pull back = 1, push forward = -1
    // going to use the Left Stick for Forward/Backward only (this will remove the noise from left/right) and
    // use the Right Stick for Left/Right turn only (this will remove those noise from the up/down)
    diffDrive.arcadeDrive(-0.55 * xboxControl.getRawAxis(1), 0.55 * xboxControl.getRawAxis(4));
  }
}
