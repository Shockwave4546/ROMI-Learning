// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ROMIChassis;

public class Forward extends CommandBase {
  // create references to HW and variables that we need inside this Command
  double distance;
  ROMIChassis chassis;

  /** Creates a new Forward. */
  public Forward(double inches) {
    // Use addRequirements() here to declare subsystem dependencies.
    distance = inches;

    // References the chassis from the RobotContainer (which we need to modify so it will be defined in there - Quizz: why there?)
    chassis = RobotContainer.chassis;
    // Basically lock out other command trying to use the chassis at the same time.
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // initialization Routine every time FORWARD command is executed
    // going to create a new RESET (master)
    chassis.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Drive forward until done
    chassis.driveForward(0.85, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return chassis.getDistance() >= distance;

  }
}
