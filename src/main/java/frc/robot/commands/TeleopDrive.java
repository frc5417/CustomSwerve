// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveBase;
import frc.robot.ModuleUtilities.ModuleGroup;
import frc.robot.subsystems.Systems;
import edu.wpi.first.wpilibj2.command.CommandBase;

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

  private final ModuleGroup m_moduleGroup;
  private final DriveBase m_driveBase;

  public TeleopDrive() {
    m_moduleGroup = Systems.moduleGroup;
    m_driveBase = Systems.driveBase;
  }

  @Override
  public void initialize() {
    m_moduleGroup.resetDrive();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xVel = Systems.getLeftJoyX() * Constants.Swerve.maxVelocity;
    double yVel = Systems.getLeftJoyY() * Constants.Swerve.maxVelocity;
    double omega = Systems.getRightJoyX() * Constants.Swerve.maxAngularVelocity;

    m_driveBase.setDriveSpeed(new ChassisSpeeds(xVel, yVel, omega));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_moduleGroup.resetDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  
}
