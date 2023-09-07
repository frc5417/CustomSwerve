// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Manipulator extends SubsystemBase {
  /** Creates a new Manipulator. */
  public CANSparkMax Intake;
  public CANSparkMax Wrist;

  private final RelativeEncoder integratedIntakeEncoder;
  private final RelativeEncoder integratedWristEncoder;

  private static final double kP = 0.4;
  private static final double kI = 0.0;
  private static final double kD = 0.0;

  public final PIDController pid = new PIDController(kP, kI, kD);
  
  private final static DutyCycleEncoder enc = new DutyCycleEncoder(Constants.MotorConstants.throughBoreEncPort);
  public double runningAverage = 0.0;

  public Manipulator() {
    Intake = new CANSparkMax(Constants.MotorConstants.intakeMotorID, MotorType.kBrushless);
    Wrist = new CANSparkMax(Constants.MotorConstants.wristMotorID, MotorType.kBrushless);

    Intake.setIdleMode(IdleMode.kCoast);
    Wrist.setIdleMode(IdleMode.kBrake);
    Intake.setInverted(Constants.MotorConstants.intakeMotorInversion);
    Wrist.setInverted(Constants.MotorConstants.wristMotorInversion);

    integratedWristEncoder = Wrist.getEncoder();
    Wrist.getPIDController();
    integratedIntakeEncoder = Intake.getEncoder();
    Intake.getPIDController();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (pid.atSetpoint()) {
      Wrist.set(0);
    } else {
      Wrist.set(pid.calculate(filteredAbsolutePosition())); //use setVoltage if no work :)
    }
  }

  public void setWrist(double pos) {
    pid.setSetpoint(pos);
  }

  public double filteredAbsolutePosition() {
    runningAverage = enc.getAbsolutePosition() * 0.1 + runningAverage * 0.9;
    return runningAverage;
  }

  public void setIntake(double speed) {
    Intake.set(speed);
  }
}
