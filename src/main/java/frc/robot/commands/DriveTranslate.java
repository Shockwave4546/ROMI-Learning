// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ROMIChassis;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveTranslate extends CommandBase {
  private final ROMIChassis m_drivetrain;
  // in inches
  private final double m_avg_distance;
  // positive makes the motors spin forward, negative makes them spin backwards
  private final double m_direction_mult;
  // the average speed we want between the motors, from zero to 1
  private final double m_avg_speed;
  // the difference between the speed of each motor to keep the robot moving straight
  private double m_speed_diff;
  // the maximum allowable speed difference between the motors
  private final double k_max_speed_diff = 0.1;

  // Integrator term to drive the difference between the L and R encoders to zero.
  // This accounts for historical inaccuracy; the longer the robot has been curving,
  // the more this part of the control loop will try to correct it
  private double m_distance_int;
  private final double k_distance_int_gain = 0.007;
  // Proportional term to drive the difference between the L and R encoders to zero.
  // This accounts for instantaneous inaccuracy; the worse the robot is currently curving,
  // the more this part of the control loop will try to correct it.
  private double m_distance_prop;
  private final double k_distance_prop_gain = 0.2;

  public DriveTranslate(ROMIChassis drivetrain, double distance, double speed) {
    m_drivetrain = drivetrain;

    // figure out if we're going forwards or backwards
    m_avg_distance = distance;
    if(m_avg_distance > 0) {
      m_direction_mult = 1;
    } else if(m_avg_distance < 0) {
      m_direction_mult = -1;
    } else {
      m_direction_mult = 0;
    }

    // make sure our speed isn't too high or low, otherwise we might try to drive
    // the motors at more than 100% or less than 0% speed
    if(speed < k_max_speed_diff / 2) {
      m_avg_speed = k_max_speed_diff / 2;
    } else if(speed > 1 - (k_max_speed_diff / 2)) {
      m_avg_speed = 1 - (k_max_speed_diff / 2);
    } else {
      m_avg_speed = speed;
    }

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.reset();
    // we have to initialize these variables, and there's no error yet
    m_distance_int = 0;
    m_distance_prop = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // calculate the encoder difference to determine proportional error
    m_distance_prop = m_drivetrain.getLeftDistance() - m_drivetrain.getRightDistance();
    // accumulate the encoder difference to determine integral error
    m_distance_int += m_distance_prop;
    // calculate the speed difference required to fix any mismatch between the encoders
    // using instantaneous (proportional) and historical (integral) error data
    m_speed_diff =  (m_distance_prop * k_distance_prop_gain) + (m_distance_int * k_distance_int_gain);

    // Make sure we're not over our motor speed difference limit
    if(m_speed_diff > k_max_speed_diff) {
      m_speed_diff = k_max_speed_diff;
    } else if(m_speed_diff < -1 * k_max_speed_diff) {
      m_speed_diff = -1 * k_max_speed_diff;
    }

    // set the speed of each motor using our drivetrain subsystem
    m_drivetrain.motorSpeed(
      (m_avg_speed - (m_speed_diff / 2)) * m_direction_mult,
      (m_avg_speed + (m_speed_diff / 2)) * m_direction_mult);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.motorSpeed(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // we're done once the average of the two encoders is the specified distance
    return (m_drivetrain.getLeftDistance() + 
            m_drivetrain.getRightDistance()) / 2 >= m_avg_distance;
  }
}
