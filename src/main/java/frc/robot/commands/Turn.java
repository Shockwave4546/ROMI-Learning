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

  /** Creates a new Turn. */
  public Turn(double degree) {
    angle = degree;
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
    chassis.Turn(0, angle);
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
    SmartDashboard.putNumber("Heading", chassis.getHeading());
    return (Math.abs(chassis.getHeading()) >= Math.abs(angle));
  }
}
