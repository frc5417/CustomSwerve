// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

// import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Manipulator extends SubsystemBase {
  /** Creates a new Manipulator. */
  public CANSparkMax Intake;
  public CANSparkMax Wrist;

  // private final RelativeEncoder integratedIntakeEncoder;
  // private final RelativeEncoder integratedWristEncoder;

  private static final double kP = 8.0; //8.3
  private static final double kI = 0.05;
  private static final double kD = 0.15;

  public final PIDController pid = new PIDController(kP, kI, kD);
  
  private final static DutyCycleEncoder enc = new DutyCycleEncoder(Constants.MotorConstants.throughBoreEncPort);
  public double runningAverage = 0.0;

  int counter = 0;

  private static double offsetPos = -3.141592653589;

  public Manipulator() {
    Intake = new CANSparkMax(Constants.MotorConstants.intakeMotorID, MotorType.kBrushless);
    Wrist = new CANSparkMax(Constants.MotorConstants.wristMotorID, MotorType.kBrushless);

    Intake.setIdleMode(IdleMode.kBrake);
    Wrist.setIdleMode(IdleMode.kBrake);
    Intake.setInverted(Constants.MotorConstants.intakeMotorInversion);
    Wrist.setInverted(Constants.MotorConstants.wristMotorInversion);

    enc.reset();
    enc.setDistancePerRotation(0.25);
    
    

    // integratedWristEncoder = Wrist.getEncoder();
    // Wrist.getPIDController();
    // integratedIntakeEncoder = Intake.getEncoder();
    // Intake.getPIDController();


    // pid.setTolerance(Constants.ManipulatorConstants.wristTolerance);
    // pid.enableContinuousInput(0, 1);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (offsetPos == -3.141592653589) {
      offsetPos = enc.getDistance();
    }

    if (Math.abs(pid.getSetpoint() - filteredAbsolutePosition()) == 0) {
      Wrist.set(0.0);
      // SmartDashboard.putNumber("At Setpoint", 1);
    } else {
      Wrist.set(MathUtil.clamp(-1 * pid.calculate(filteredAbsolutePosition()), -1, 1)); //use setVoltage if no work :)
      // SmartDashboard.putNumber("At Setpoint", 0);
      SmartDashboard.putNumber("PID OUT: ", MathUtil.clamp(-1 * pid.calculate(filteredAbsolutePosition()), -0.7, 0.7));
    }
    SmartDashboard.putNumber("Encoder", filteredAbsolutePosition());
    // SmartDashboard.putNumber("WristSetpoint", pid.getSetpoint());
  }

  public void setWrist(double pos) {
    pid.setSetpoint(pos);
  }

  public void setWristSpeed(double speed) {
    Wrist.set(0.25 * speed);
  }

  public double filteredAbsolutePosition() {
    runningAverage = (Math.abs(enc.getDistance() - offsetPos)) * 0.9 + runningAverage * 0.1;
    return runningAverage;
  }

  public void setIntake(double speed) {
    Intake.set(speed * 1.0);
  }

  // public double positionMapper(double unitJoyRange) {
  //   double slope = (Constants.ManipulatorConstants.wristMax - Constants.ManipulatorConstants.wristMin) / 2.0;
  //   double intercept = Constants.ManipulatorConstants.wristMin;
  //   return (slope * (unitJoyRange + 1.0)) + intercept;
  // }
}
