// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants;

public class ServoArm extends SubsystemBase {
  
  Servo arm;

  /** Creates a new ServoArm. */
  public ServoArm() {
    arm = new Servo(Constants.Servo_Port);

    // put the arm on the dashboard
    addChild("Servo Arm:", arm);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Create the reset command to reset the left and right encoders and also the gyro too
  public void reset() {
    arm.setPosition(0);
  }

  public void moveUp(double pos) {
    arm.setPosition(pos);
  }
}
