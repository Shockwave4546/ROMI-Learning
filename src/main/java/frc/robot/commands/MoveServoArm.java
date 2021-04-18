// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ServoArm;

public class MoveServoArm extends CommandBase {
  ServoArm armServo;
  double pos;
  int direction = 1;

  /** Creates a new MoveServoArm. */
  public MoveServoArm() {
    // Use addRequirements() here to declare subsystem dependencies.
    armServo = RobotContainer.armServo;
    addRequirements(armServo);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // For now, don't reset the arm position
    // armServo.reset();
    pos = 0.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get the Position From SmartDashboard for now....can change to NetworkTables down the road
    // pos = SmartDashboard.getNumber("ArmPosition", 0.0);

    // sweep the pos range - Servo can only be 0-1 position range
    if ((pos > 0.99) || (pos < 0)) {
      direction = -1 * direction;
    }
    pos = pos + (0.01 * direction);
    SmartDashboard.putNumber("ArmPosition", pos);
    armServo.moveUp(pos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armServo.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
