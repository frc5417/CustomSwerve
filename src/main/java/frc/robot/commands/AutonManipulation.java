// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;
import edu.wpi.first.wpilibj2.command.Command;


public class AutonManipulation extends Command {
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

class ElevationUp extends Command {
  private Elevator m_elevator;
  public ElevationUp(Elevator elevator) {
      m_elevator = elevator;
  }
  int counter = 0;
  boolean ended = false;

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if(counter < 3500/20){  
      counter += 1;  
      m_elevator.UpAndAway(-0.375);  
    } else {
      m_elevator.UpAndAway(0);
      ended = true; 
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    if (ended == true){
      return true;
    } else {
      return false;
    }
  }
}

class ElevationDown extends Command {
  private final Elevator m_elevator;
  public ElevationDown(Elevator elevator) {
      m_elevator = elevator;
  }
  int counter = 0;
  boolean ended = false;

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if(counter < 3000/20){  
      counter += 1;  
      m_elevator.UpAndAway(0.375);  
    } else {
      m_elevator.UpAndAway(0);
      ended = true;
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    if (ended == true){
      return true;
    } else {
      return false;
    }
  }
}

class Intake extends Command {
  private final Manipulator m_manipulator;
  public Intake(Manipulator manipulator) {
      m_manipulator = manipulator;
  }
  int counter = 0;
  boolean ended = false;

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if(counter < 1000/20){  
      counter += 1;  
      m_manipulator.setIntake(0.5);  
    } else {
      m_manipulator.setIntake(0);
      ended = true;
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    if (ended == true){
      return true;
    } else {
      return false;
    }
  }
}

class Outtake extends Command {
  private final Manipulator m_manipulator;
  public Outtake(Manipulator manipulator) {
      m_manipulator = manipulator;
  }
  int counter = 0;
  boolean ended = false;

  @Override
  public void initialize() {}

  @Override
  public void execute() {  
    if(counter < 1000/20){  
      counter += 1;  
      m_manipulator.setIntake(-0.5);  
    } else {
      m_manipulator.setIntake(0);
      ended = true;
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    if (ended == true){
      return true;
    } else {
      return false;
    }
  }
}