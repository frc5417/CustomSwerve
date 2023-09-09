// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.RobotContainer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Module;

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
  private final Manipulator m_manipulator;
  private final Elevator m_elevator;

  private int cnt = 0;

  double prev_omega = 0;
  double prev_xVel = 0;
  double prev_yVel = 0;
 
  int counter = 0;

  public TeleopDrive(DriveBase driveBase, Manipulator manipulator, Elevator elevator) {
    m_driveBase = driveBase;
    m_manipulator = manipulator;
    m_elevator = elevator;
  }

  @Override
  public void initialize() {
    // m_driveBase.resetDrive();
    // Module.ModuleState[] temp = new Module.ModuleState[4];

    // for (int i = 0; i < 4; i++)
    //   temp[i] = new Module.ModuleState(0, Constants.MotorConstants.motorDegrees[i] * (Math.PI/180));

    // m_driveBase.setHardStates(temp);
    RobotContainer.setLEDsOn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (counter++ <= 60)
    //   return;
    
    double xVel = (RobotContainer.getDriverLeftJoyX() * 0.45) + (prev_xVel * 0.55); //* Constants.Swerve.maxVelocity;
    double yVel = (RobotContainer.getDriverLeftJoyY() * 0.45) + (prev_yVel * 0.55); //* Constants.Swerve.maxVelocity;
    double omega = (RobotContainer.getDriverRightJoyX() * 0.45) + (prev_omega * 0.55); //* Constants.Swerve.maxAngularVelocity;

    prev_xVel = xVel;
    prev_yVel = yVel;
    prev_omega = omega;


    SmartDashboard.putNumber("X-Vel Input", xVel);
    SmartDashboard.putNumber("Y-Vel Input", yVel);
    SmartDashboard.putNumber("Omega Vel Input", omega);

    // if (cnt++ % 50 == 0) 
      // System.out.printf("xVelocity: %f, yVelocity: %f, AngularVel: %f", xVel, yVel, omega);

    // m_driveBase.setDriveSpeed(new ChassisSpeeds(0*Constants.Swerve.XPercentage, 0.25*Constants.Swerve.YPercentage, 0*Constants.Swerve.angularPercentage));
    m_driveBase.setDriveSpeed(new ChassisSpeeds(xVel * Constants.Swerve.XPercentage, yVel * Constants.Swerve.YPercentage, omega * Constants.Swerve.angularPercentage));
    m_elevator.UpAndAway(RobotContainer.getManipulatorLeftJoyY());
    if (RobotContainer.getManipulatorLeftTrigger() > 0 && RobotContainer.getManipulatorRightTrigger() <= 0) {
      m_manipulator.setIntake(RobotContainer.getManipulatorLeftTrigger());
    } else if (RobotContainer.getManipulatorLeftTrigger() <= 0 && RobotContainer.getManipulatorRightTrigger() > 0) {
      m_manipulator.setIntake(RobotContainer.getManipulatorRightTrigger() * -1);
    } else {
      m_manipulator.setIntake(0.0);
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
