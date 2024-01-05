// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  public CANSparkMax Motor1;
  public CANSparkMax Motor2;

  // private final RelativeEncoder integratedMotor1Encoder;
  // private final RelativeEncoder integratedMotor2Encoder;


  public Elevator() {
    Motor1 = new CANSparkMax(Constants.MotorConstants.elevatorMotorIDS[0], MotorType.kBrushless);
    Motor2 = new CANSparkMax(Constants.MotorConstants.elevatorMotorIDS[1], MotorType.kBrushless);

    Motor1.setIdleMode(IdleMode.kBrake);
    Motor2.setIdleMode(IdleMode.kBrake);
    Motor1.setInverted(Constants.MotorConstants.elevatorMotorInversions[0]);
    Motor2.setInverted(Constants.MotorConstants.elevatorMotorInversions[1]);

    // integratedMotor1Encoder = Motor1.getEncoder();
    // Motor1.getPIDController();
    // integratedMotor2Encoder = Motor2.getEncoder();
    // Motor2.getPIDController();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void UpAndAway(double speed) {
    Motor1.set(speed*0.25);
    Motor2.set(speed*0.25);
  }
}
