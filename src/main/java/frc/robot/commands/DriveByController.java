// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ROMIChassis;

public class DriveByController extends CommandBase {

  ROMIChassis chassis;
  XboxController xboxControl;

  /** Creates a new DriveByController. */
  public DriveByController() {
    chassis = RobotContainer.chassis;
    xboxControl = RobotContainer.xboxControl;
    
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
    // Put the latest Heading (angle) on the Smastdashboard
    SmartDashboard.putNumber("Heading: ", chassis.getHeading());
    chassis.DBC(xboxControl);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
