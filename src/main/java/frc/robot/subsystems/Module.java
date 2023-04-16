package frc.robot.subsystems;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import com.revrobotics.CANSparkMax;

import frc.robot.Constants;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;


public class Module {
  /** Creates a new Module. */

  public CANSparkMax angleMotor;
  private CANSparkMax driveMotor;

  private final RelativeEncoder integratedDriveEncoder;
  private final RelativeEncoder integratedAngleEncoder;

  private final int moduleNum; // ZERO INDEXED

  private static final double kP = 0.4;
  private static final double kI = 0.0;
  private static final double kD = 0.005;

  public final PIDController pid = new PIDController(kP, kI, kD);

  private Boolean invertDriveSpeed = false;
  
  private WPI_CANCoder _CANCoder;

  public Module(int module, boolean inverted) {
    
    this.moduleNum = module;

     /* Angle Motor Config */
     angleMotor = new CANSparkMax(Constants.MotorConstants.angleMotorIDS[this.moduleNum], MotorType.kBrushless);
     configAngleMotor();

     integratedAngleEncoder = angleMotor.getEncoder();
     angleMotor.getPIDController();

     _CANCoder = new WPI_CANCoder(Constants.MotorConstants.CANCoderID[this.moduleNum], "canivore");

    //  _CANCoder.setPositionToAbsolute(0);
     _CANCoder.configAllSettings(returnCANConfig());
     _CANCoder.setPosition(0);
    
     /* Drive Motor Config */
     driveMotor = new CANSparkMax(Constants.MotorConstants.driveMotorIDS[this.moduleNum], MotorType.kBrushless);
     configDriveMotor();

     integratedDriveEncoder = driveMotor.getEncoder();
     driveMotor.getPIDController();
     

    pid.enableContinuousInput(0, Math.PI * 2);

    this.invertDriveSpeed = inverted;
    // if(_CANCoder.getMagnetFieldStrength() != MagnetFieldStrength.Good_GreenLED) {
      // throw new RuntimeException("CANCoder on Module #" + Integer.valueOf(this.moduleNum).toString() + " is not green!");
    // }
  }

  public static class ModuleState {
    private final double m_vel;
    private final double m_dir;

    public ModuleState(double vel, double dir) {
        m_vel = vel;
        m_dir = dir;
    }

    public double getVel() {
        return m_vel;
    }

    public double getDir() {
        return m_dir;
    }
}

int cnt = 0;

  public void setSpeedAndAngle(ModuleState targetState) {
    double x = setAngle(targetState.getDir());
    double y = setDriveSpeed(targetState.getVel());

    if (++cnt % 50 == 0) {
      System.out.printf("Set module %d angle to %f, speed to %f\n", this.moduleNum, x, y);
    }
  }

  //angle to normalize between 0 and 2PI RAD
  public double normalizeAngle(double angle) {
    double fixedAngle = angle;
    while (fixedAngle > (2*Math.PI)) { 
      fixedAngle -= (2*Math.PI); 
    }
    while (fixedAngle < 0.0 ) { 
      fixedAngle += (2*Math.PI); 
    }
    return fixedAngle;
  }

  public double setDriveSpeed(double speed) {
    int invertMultiplier = 1;
    if (invertDriveSpeed) { 
      invertMultiplier = -1; 
    }

    double x = speed * invertMultiplier;
    
    driveMotor.set(x);
    return x;
  }

  public double getAngleInRadians() { 
    return _CANCoder.getPosition() * (Math.PI/180.0);
  }

  public double getAngle() {
    return _CANCoder.getPosition();
  }

  public double getDriveVelocity() {
    return integratedDriveEncoder.getVelocity();
  }

  public double getAngularVelocity() {
    return integratedAngleEncoder.getVelocity();
  }

  public double setAngle(double angle_in_rad) {
    // //code to make the angle motor turn the least amount possible and drive direction if necessary
    // double targetAngle = angle_in_rad + Constants.MotorConstants.angleOffsets[this.moduleNum - 1];
    // double cudoublerrentAngle = getAngle();
    // double normalDifference = currentAngle - targetAngle;
    // double difference180 = currentAngle - normalizeAngle(targetAngle+180.0);

    // //if going to targetAngle + 180 degrees is not less than the distance of going just to targetAngle
    // //then turn normally and also do not invert the motor direction
    // if (Math.abs(normalDifference) <= Math.abs(difference180)) {
    //   pid.setSetpoint(targetAngle); // angles are in TRUE BEARING ( angles are negated )
    // } else {
    //   pid.setSetpoint(targetAngle+180.0); // angles are in TRUE BEARING ( angles are negated )
    //   invertSpeed();
    // }

    double x = (angle_in_rad);

    pid.setSetpoint(angle_in_rad);

    if (Math.abs(this.pid.getSetpoint() - this.getAngleInRadians()) > (Constants.MotorConstants.degTolerance*(Math.PI/180))) {
      this.angleMotor.set(MathUtil.clamp(this.pid.calculate(this.getAngleInRadians()), -0.8, 0.8));
    } else {
      this.angleMotor.set(0.0);
    }

    return x;
  }

  public void resetDriveAngleEncoder() {
    _CANCoder.close();
  }

  public CANCoderConfiguration returnCANConfig() {
    CANCoderConfiguration canConfig = new CANCoderConfiguration();
    canConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    canConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
    canConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    return canConfig;
  }

  private void configAngleMotor() {
    angleMotor.restoreFactoryDefaults();
    // CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
    // angleMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
    // angleMotor.setInverted(Constants.Swerve.angleInvert);
    angleMotor.setIdleMode(Constants.Swerve.angleNeutralMode);
    // integratedAngleEncoder.setPositionConversionFactor(Constants.Swerve.angleConversionFactor);
    // angleController.setP(Constants.Swerve.angleKP);
    // angleController.setI(Constants.Swerve.angleKI);
    // angleController.setD(Constants.Swerve.angleKD);
    // angleController.setFF(Constants.Swerve.angleKFF);
    // angleMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
    angleMotor.burnFlash();
  }

  private void configDriveMotor() {
    driveMotor.restoreFactoryDefaults();
    // CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kVelocityOnly);
    // driveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
    // driveMotor.setInverted(Constants.Swerve.driveInvert);
    driveMotor.setIdleMode(Constants.Swerve.driveNeutralMode);
    // driveEncoder.setVelocityConversionFactor(Constants.Swerve.driveConversionVelocityFactor);
    // driveController.setP(Constants.Swerve.angleKP);
    // driveController.setI(Constants.Swerve.angleKI);
    // driveController.setD(Constants.Swerve.angleKD);
    // driveController.setFF(Constants.Swerve.angleKFF);
    // driveMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
    driveMotor.burnFlash();
  }
}
