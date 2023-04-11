// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Compute;
import frc.robot.subsystems.Module;
import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** An example command that uses an example subsystem. */
public class TeleopDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xVel = RobotContainer.getLeftJoyX() * Constants.Swerve.maxVelocity;
    double yVel = RobotContainer.getLeftJoyY() * Constants.Swerve.maxVelocity;
    double omega = RobotContainer.getRightJoyX() * Constants.Swerve.maxAngularVelocity;

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xVel, yVel, omega);

    RobotContainer.m_driveBase.setDriveSpeed(chassisSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_driveBase.resetModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  
}
