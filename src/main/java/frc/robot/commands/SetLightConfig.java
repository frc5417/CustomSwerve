// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LightsControl;

public class SetLightConfig extends Command {
  private final LightsControl lightsControl;
  private int configNum;

  /** Creates a new SetLightConfig. */
  public SetLightConfig(LightsControl subsystem, int configNum) {
    this.lightsControl = subsystem;
    this.configNum = configNum;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(lightsControl);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lightsControl.setLightConfig(configNum);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}