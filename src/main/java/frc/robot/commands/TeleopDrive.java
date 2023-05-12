// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
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

  double prev_omega = 0;
  double prev_xVel = 0;
  double prev_yVel = 0;
 
  int counter = 0;

  public TeleopDrive(DriveBase driveBase) {
    m_driveBase = driveBase;
  }

  @Override
  public void initialize() {
    // Module.ModuleState[] temp = new Module.ModuleState[4];

    // for (int i = 0; i < 4; i++)
    //   temp[i] = new Module.ModuleState(0, Constants.MotorConstants.motorDegrees[i] * (Math.PI/180));

    // m_driveBase.setHardStates(temp);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double xVel = ((RobotContainer.getLeftJoyX()) * 0.45 * Constants.Swerve.maxSetVelocity) + (prev_xVel * 0.55); //* Constants.Swerve.maxVelocity;
    double yVel = ((RobotContainer.getLeftJoyY()) * 0.45 * Constants.Swerve.maxSetVelocity) + (prev_yVel * 0.55); //* Constants.Swerve.maxVelocity;
    double omega = ((RobotContainer.getRightJoyX()) * 0.45 * Constants.Swerve.maxSetAngularVelocity) + (prev_omega * 0.55); //* Constants.Swerve.maxAngularVelocity;

    prev_xVel = xVel;
    prev_yVel = yVel;
    prev_omega = omega;

    SmartDashboard.putNumber("X-Vel Input", xVel);
    SmartDashboard.putNumber("Y-Vel Input", yVel);
    SmartDashboard.putNumber("Omega Vel Input", omega);

    ChassisSpeeds newSpeed = new ChassisSpeeds(xVel, yVel, omega);
    m_driveBase.setDriveSpeed(newSpeed);

    if (counter++ % 50 == 0) {
      System.out.println(newSpeed);
    }
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
