// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import javax.swing.text.Utilities;
import frc.robot.subsystems.RobotContainer;
import frc.robot.Constants;

public class AutonManipulation extends CommandBase {
  public AutonManipulation() {}

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}

class ElevationUp extends CommandBase {
  private final Elevator m_elevator;
  public ElevationUp(Elevator elevator) {
      m_elevator = elevator;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    final long initTime = RobotContainer.getFPGATime();
    while (RobotContainer.getFPGATime() - initTime <= Constants.Swerve.CommandDuration) {     
      m_elevator.UpAndAway(-0.375);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_elevator.UpAndAway(0);
  }

  @Override
  public boolean isFinished() {
      return false;
  }
}

class ElevationDown extends CommandBase {
  private final Elevator m_elevator;
  public ElevationDown(Elevator elevator) {
      m_elevator = elevator;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    final long initTime = RobotContainer.getFPGATime();
    while (RobotContainer.getFPGATime() - initTime <= Constants.Swerve.CommandDuration) {     
      m_elevator.UpAndAway(0.375);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_elevator.UpAndAway(0);
  }

  @Override
  public boolean isFinished() {
      return false;
  }
}

class Intake extends CommandBase {
  private final Manipulator m_manipulator;
  public Intake(Manipulator manipulator) {
      m_manipulator = manipulator;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    final long initTime = RobotContainer.getFPGATime();
    while (RobotContainer.getFPGATime() - initTime <= Constants.Swerve.CommandDuration) {     
      m_manipulator.setIntake(0.375);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_manipulator.setIntake(0);
  }

  @Override
  public boolean isFinished() {
      return false;
  }
}

class Outtake extends CommandBase {
  private final Manipulator m_manipulator;
  public Outtake(Manipulator manipulator) {
      m_manipulator = manipulator;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    final long initTime = RobotContainer.getFPGATime();
    while (RobotContainer.getFPGATime() - initTime <= Constants.Swerve.CommandDuration) {     
      m_manipulator.setIntake(-0.375);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_manipulator.setIntake(0);
  }

  @Override
  public boolean isFinished() {
      return false;
  }
}