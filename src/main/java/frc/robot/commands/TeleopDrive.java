// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.RobotContainer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** An example command that uses an example subsystem. */
public class TeleopDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */

  // Called when the command is initially scheduled.

  private final DriveBase m_driveBase;

  private int cnt = 0;

  public TeleopDrive(DriveBase driveBase) {
    m_driveBase = driveBase;
  }

  @Override
  public void initialize() {
    m_driveBase.resetDrive();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xVel = RobotContainer.getLeftJoyX(); //* Constants.Swerve.maxVelocity;
    double yVel = RobotContainer.getLeftJoyY(); //* Constants.Swerve.maxVelocity;
    double omega = RobotContainer.getRightJoyX(); //* Constants.Swerve.maxAngularVelocity;

    SmartDashboard.putNumber("X-Vel Input", xVel);
    SmartDashboard.putNumber("Y-Vel Input", yVel);
    SmartDashboard.putNumber("Omega Vel Input", omega);

    if (cnt++ % 50 == 0) 
      System.out.printf("xVelocity: %f, yVelocity: %f, AngularVel: %f", xVel, yVel, omega);

    m_driveBase.setDriveSpeed(new ChassisSpeeds(xVel, yVel, omega));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveBase.resetDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  
}
