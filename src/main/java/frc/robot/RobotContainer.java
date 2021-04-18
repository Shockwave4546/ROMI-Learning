// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ROMIChassis;
import frc.robot.subsystems.ServoArm;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveByController;
import frc.robot.commands.DriveTranslate;
import frc.robot.commands.Forward;
import frc.robot.commands.Turn;
import frc.robot.commands.MoveServoArm;
// import frc.robot.commands.AutoDrive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public static final ROMIChassis chassis = new ROMIChassis();
  public static final ServoArm armServo = new ServoArm();
  // Add and xbox controller
  public static final XboxController xboxControl = new XboxController(Constants.ControllerPort);

  // private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //SmartDashboard.putData("Forward", new Forward(15.2));
    //SmartDashboard.putData("L30-Turn", new Turn(-30.0));
    //SmartDashboard.putData("R30-Turn", new Turn(30.0));
    //SmartDashboard.putData("L80-Turn", new Turn(-75.0));
    SmartDashboard.putData("Nick", new DriveTranslate(chassis, 10, 0.7));
    SmartDashboard.putData("GONick", new DriveTranslate(chassis, 14, 0.4));

    SmartDashboard.putData("ZZZZZ", new Turn(10));
    SmartDashboard.putData("YYYYY", new Turn(-10));

    SmartDashboard.putData("GoGo GoGo", new SequentialCommandGroup(
      new Forward(15.6),
      new Turn(-71.4),
      new Forward(15.2),
      new Turn(-68.5),
      new Forward(9.2),
      new Turn(-33.0),
      new Forward(8.6),
      new Turn(27.3),
      new Forward(7.4),
      new Turn(72.4),
      new Forward(13.6),
      new Turn(69.0),
      new Forward(15.3)
    ));

    SmartDashboard.putData("Arm to", new MoveServoArm());
    // SmartDashboard.putNumber("ArmPosition", 0.25);

    //m_chooser.setDefaultOption("Forward 10in", new Forward(10.0));
    //m_chooser.addOption("Forward 1in", new Forward(1.0));
    //SmartDashboard.putData(m_chooser);

// Moved to TeleOp Only so it wont gets in the way
//    // Add XBox Controller as the default drive by controller
//    chassis.setDefaultCommand((Command) new DriveByController());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // return m_autoCommand;
    // return new PrintCommand("message");
    // Not doing Autonomous Drive for now, just do inline command groupings instead
    // return new AutoDrive();
    return new PrintCommand("Msg");
  }

  //
  public Command gTeleOpOnly() {
    // Add XBox Controller as the default drive by controller
    return (Command) new DriveByController();
  }
}
