// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Module;
import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.math.MathUtil;

/** An example command that uses an example subsystem. */
public class DriveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Module m_module1;
  private final Module m_module2;
  private final Module m_module3;
  private final Module m_module4;
  
  private int angle = 0;
  private int counter = 0;

  

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveCommand(Module mod1, Module mod2, Module mod3, Module mod4) {
    m_module1 = mod1;
    m_module2 = mod2;
    m_module3 = mod3;
    m_module4 = mod4;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_module1, m_module2, m_module3, m_module4);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_module1.setAngle(0);
    m_module2.setAngle(0);
    // m_module3.setAngle(Math.PI/4);
    m_module4.setAngle(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    this.angleRun();

    if ((counter++ % 100) == 0) {
      m_module1.setAngle(this.angle);
      m_module2.setAngle(this.angle);
      // m_module3.setAngle(this.angle);
      m_module4.setAngle(this.angle);

      m_module1.setDriveSpeed(0.3);
      m_module2.setDriveSpeed(0.3);
      // m_module3.setDriveSpeed(1);
      m_module4.setDriveSpeed(0.3);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_module1.resetDriveAngleEncoder();
    // m_module2.resetDriveAngleEncoder();
    // m_module3.resetDriveAngleEncoder();
    // m_module4.resetDriveAngleEncoder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void angleRun() {
    if (!m_module1.pid.atSetpoint()) {
      m_module1.angleMotor.set(MathUtil.clamp(m_module1.pid.calculate(m_module1.getAngleInRadians()), -1, 1));
    }
    if (!m_module2.pid.atSetpoint()) {
      m_module2.angleMotor.set(MathUtil.clamp(m_module2.pid.calculate(m_module2.getAngleInRadians()), -1, 1));
    }
    if (!m_module3.pid.atSetpoint()) {
      m_module3.angleMotor.set(MathUtil.clamp(m_module3.pid.calculate(m_module3.getAngleInRadians()), -1, 1));
    }
    if (!m_module4.pid.atSetpoint()) {
      m_module4.angleMotor.set(MathUtil.clamp(m_module4.pid.calculate(m_module4.getAngleInRadians()), -1, 1));
    }


  }
}
