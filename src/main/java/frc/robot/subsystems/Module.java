// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import frc.robot.Constants;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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

  private CANCoder angleEncoder;

  private final int module_num;

  private static final double kP = 0.1;
  private static final double kI = 0.0;
  private static final double kD = 0.005;

  public final PIDController pid = new PIDController(kP, kI, kD);

  private Boolean invertDriveSpeed = false;
  

  public Module(int module) {
    this.module_num = module;
    angleEncoder = new CANCoder(Constants.MotorConstants.pwmIDS[this.module_num-1],"canivore");
    //angleEncoder.configFactoryDefault();
    //angleEncoder.configAllSettings(returnCANConfig());
     /* Angle Motor Config */
     angleMotor = new CANSparkMax(Constants.MotorConstants.angleMotorIDS[this.module_num-1], MotorType.kBrushless);
     integratedAngleEncoder = angleMotor.getEncoder();
     angleController = angleMotor.getPIDController();
     /*angleController.setP(Constants.MotorConstants.angleMotorPID[this.module_num-1][0]);
     angleController.setI(Constants.MotorConstants.angleMotorPID[this.module_num-1][1]);
     angleController.setD(Constants.MotorConstants.angleMotorPID[this.module_num-1][2]);*/
    //  configAngleMotor();
    
     /* Drive Motor Config */
     driveMotor = new CANSparkMax(Constants.MotorConstants.driveMotorIDS[this.module_num-1], MotorType.kBrushless);
     integratedDriveEncoder = driveMotor.getEncoder();
     driveController = driveMotor.getPIDController();

     angleEncoder.setPosition(0);
     //angleEncoder.reset();
    //  configDriveMotor();

    pid.enableContinuousInput(0, Math.PI * 2);
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

  public void invertSpeed() {
    invertDriveSpeed = !invertDriveSpeed;
  }

  //angle to normalize between 0 and 360 degrees
  public double normalizeAngle(double angle) {
    double fixedAngle = angle;
    if (fixedAngle > 360.0) { fixedAngle -= 360.0; }
    else if (fixedAngle < 0.0 ) { fixedAngle += 360.0; }
    return fixedAngle;
  }

  public void setDriveSpeed(double speed) {
    int invertMultiplier = 1;
    if (invertDriveSpeed) { invertMultiplier = -1; }
    driveMotor.set(speed * invertMultiplier);
  }

  public double getAngleInRadians() {
    return (angleEncoder.getAbsolutePosition()) * Math.PI / 180.0;
  }

  public double getAngle() {
    return angleEncoder.getAbsolutePosition();
  }

  public void setAngle(double angle_in_rad) {
    //code to make the angle motor turn the least amount possible and drive direction if necessary
    double targetAngle = angle_in_rad + Constants.MotorConstants.angleOffsets[this.module_num - 1];
    double currentAngle = getAngle();
    double normalDifference = currentAngle - targetAngle;
    double difference180 = currentAngle - normalizeAngle(targetAngle+180.0);

    //if going to targetAngle + 180 degrees is not less than the distance of going just to targetAngle
    //then turn normally and also do not invert the motor direction
    if (Math.abs(normalDifference) <= Math.abs(difference180)) {
      pid.setSetpoint(targetAngle); // angles are in TRUE BEARING ( angles are negated )
    } else {
      pid.setSetpoint(targetAngle+180.0); // angles are in TRUE BEARING ( angles are negated )
      invertSpeed();
    }
    
  }

  public void resetDriveAngleEncoder() {
    angleEncoder.setPosition(0);
    
    //angleEncoder.reset();
    //angleEncoder.DestroyObject();
  }

  public CANCoderConfiguration returnCANConfig() {
    CANCoderConfiguration canConfig = new CANCoderConfiguration();
    canConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    canConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    canConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    return canConfig;
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
