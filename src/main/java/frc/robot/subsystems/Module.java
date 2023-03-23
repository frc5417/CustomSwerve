// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderFaults;
import com.ctre.phoenix.sensors.CANCoderStickyFaults;
import com.ctre.phoenix.sensors.MagnetFieldStrength;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Module extends SubsystemBase {
  /** Creates a new Module. */

  public CANSparkMax angleMotor;
  private CANSparkMax driveMotor;

  private final SparkMaxPIDController driveController;
  private final SparkMaxPIDController angleController;

  private final RelativeEncoder integratedDriveEncoder;
  private final RelativeEncoder integratedAngleEncoder;

  private DutyCycleEncoder angleEncoder;

  private final int module_num;

  private static final double kP = 0.4;
  private static final double kI = 0.0;
  private static final double kD = 0.005;

  public final PIDController pid = new PIDController(kP, kI, kD);
  
  WPI_CANCoder _CANCoder;
  CANCoderConfiguration _canCoderConfiguration = new CANCoderConfiguration();

  public Module(int module) {
    this.module_num = module;
    angleEncoder = new DutyCycleEncoder(Constants.MotorConstants.pwmIDS[this.module_num-1]);
    System.out.printf("Initial encoder value: %f\n", angleEncoder.getAbsolutePosition());
     /* Angle Motor Config */
     angleMotor = new CANSparkMax(Constants.MotorConstants.angleMotorIDS[this.module_num-1], MotorType.kBrushless);
     integratedAngleEncoder = angleMotor.getEncoder();
     angleController = angleMotor.getPIDController();

     _CANCoder = new WPI_CANCoder(Constants.MotorConstants.CANCoderID[this.module_num-1], "canivore");
    //  configAngleMotor();
    
     /* Drive Motor Config */
     driveMotor = new CANSparkMax(Constants.MotorConstants.driveMotorIDS[this.module_num-1], MotorType.kBrushless);
     integratedDriveEncoder = driveMotor.getEncoder();
     driveController = driveMotor.getPIDController();

     angleEncoder.reset();
     _CANCoder.setPosition(0);
    //  configDriveMotor();

    pid.enableContinuousInput(0, Math.PI * 2);

    angleMotor.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // if (!pid.atSetpoint() ) {
    //   angleMotor.set(MathUtil.clamp(pid.calculate(this.getAngleInRadians()), -0.3, 0.3));
    // } else {
    //   angleMotor.set(0);
    // }

  }

  public void setDriveSpeed(double speed) {
    if (this.module_num == 3 || this.module_num == 4) {
      driveMotor.set(-speed);
    } else {
      driveMotor.set(speed);
    }
  }

  public double getAngleInRadians() {
    // return (angleEncoder.getAbsolutePosition() - angleEncoder.getPositionOffset()) * Math.PI * 2;
    return _CANCoder.getPosition() * (Math.PI/180.0);
  }

  public double getAngle() {
    // return angleEncoder.getAbsolutePosition() - angleEncoder.getPositionOffset();
    return _CANCoder.getPosition();
  }

  public void setAngle(double angle_in_rad) {

    pid.setSetpoint(angle_in_rad + Constants.MotorConstants.angleOffsets[this.module_num - 1]); // angles are in TRUE BEARING ( angles are negated )

  }
  public void resetDriveAngleEncoder() {
    angleEncoder.reset();
    angleEncoder.close();
    _CANCoder.close();
  }

  // private void configAngleMotor() {
  //   angleMotor.restoreFactoryDefaults();
  //   CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
  //   angleMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
  //   angleMotor.setInverted(Constants.Swerve.angleInvert);
  //   angleMotor.setIdleMode(Constants.Swerve.angleNeutralMode);
  //   integratedAngleEncoder.setPositionConversionFactor(Constants.Swerve.angleConversionFactor);
  //   angleController.setP(Constants.Swerve.angleKP);
  //   angleController.setI(Constants.Swerve.angleKI);
  //   angleController.setD(Constants.Swerve.angleKD);
  //   angleController.setFF(Constants.Swerve.angleKFF);
  //   angleMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
  //   angleMotor.burnFlash();
  // }

  // private void configDriveMotor() {
  //   driveMotor.restoreFactoryDefaults();
  //   CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kVelocityOnly);
  //   driveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
  //   driveMotor.setInverted(Constants.Swerve.driveInvert);
  //   driveMotor.setIdleMode(Constants.Swerve.driveNeutralMode);
  //   driveEncoder.setVelocityConversionFactor(Constants.Swerve.driveConversionVelocityFactor);
  //   driveController.setP(Constants.Swerve.angleKP);
  //   driveController.setI(Constants.Swerve.angleKI);
  //   driveController.setD(Constants.Swerve.angleKD);
  //   driveController.setFF(Constants.Swerve.angleKFF);
  //   driveMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
  //   driveMotor.burnFlash();
  //   driveEncoder.setPosition(0.0);
  // }
}
