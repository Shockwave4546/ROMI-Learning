// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ROMIChassis;

public class Turn extends CommandBase {
  double angle;
  ROMIChassis chassis;
  // When turnSpeed at 0.30, turned Angle is off by about 1 degree so probably slowest we want to go
  // When turnSpeed at 0.45, turned Angle is off by 1-4 degrees so probably fastest we want to go
  static final double turnSpeed = 0.38; 
  // When input Angle is negative, we wanted to turn LEFT, otherwise turn RIGHT (save 180 turn time)
  // So we will flip the turnDirection based on input angle.  
  double turnDirection;

  /** Creates a new Turn. */
  public Turn(double degree) {
    angle = degree;
    // Flip the Turn Direction based on angle pos/neg
    if (angle < 0) {
      turnDirection = -1;
    } else {
      turnDirection = 1;
    }
    chassis = RobotContainer.chassis;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    chassis.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double enda;
    enda = chassis.getHeading();
      if (angle > 0) {
        if (enda >= angle) {
          chassis.stop();
        } else {
          chassis.Turn(0, turnDirection * turnSpeed);
        }
      } else {
        if (angle >= enda) {
          chassis.stop();
        } else {
          chassis.Turn(0, turnDirection * turnSpeed);
        }
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putNumber("Distance", chassis.getDistance());
    SmartDashboard.putNumber("Heading", chassis.getHeading());
    chassis.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putNumber("Distance", chassis.getDistance());
    SmartDashboard.putNumber("Heading", chassis.getHeading());
    // return (Math.abs(chassis.getHeading()) >= Math.abs(angle));
    if (angle > 0) {
      return (chassis.getHeading() >= angle);
    } else {
      return (angle >= chassis.getHeading());
    }
  }
}
