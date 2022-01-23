// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Turn extends CommandBase {
  private final Drivetrain m_drive;
  private final double m_speed;
  private final double m_target;

  /** Creates a new Turn. */
  public Turn(double speed, double targetDegrees, Drivetrain drive) {
    m_target = targetDegrees;
    m_speed = speed;
    m_drive = drive;
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.arcadeDrive(0,0);
    m_drive.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double error = m_target - m_drive.getGyroAngleZ();
    double degreesChange = error * Constants.kP;
    m_drive.arcadeDrive(m_speed, degreesChange);
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
