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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Module {
  /** Creates a new Module. */

  public CANSparkMax angleMotor;
  private CANSparkMax driveMotor;
  
  private double oldWheelPos = 0;

  private double currentSpeed = 0;
  private double currentAngle = 0;

  private final RelativeEncoder integratedDriveEncoder;
  private final RelativeEncoder integratedAngleEncoder;

  private final int moduleNum; // ZERO INDEXED

  private static final double kP = 0.2;
  private static final double kI = 0.0;
  private static final double kD = 0.005;

  double lastSpeed = 0;
  double lastdir = 0;

  public final PIDController pid = new PIDController(kP, kI, kD);

  private Boolean invertDriveSpeed = false;
  
  private WPI_CANCoder _CANCoder;

  double curDrive = 0;

  private final Translation2d m_distFromCenter;

  int cnt = 0;


  /*
   * 
   * @param 
   * @param
   * @param the x and y distance of the module from the robot's center of rotation
   * 
   */

  public Module(int module, boolean inverted, Translation2d distFromCenter) {
    
    this.moduleNum = module;

     /* Angle Motor Config */
     angleMotor = new CANSparkMax(Constants.MotorConstants.angleMotorIDS[this.moduleNum], MotorType.kBrushless);
     configAngleMotor();

     integratedAngleEncoder = angleMotor.getEncoder();
     angleMotor.getPIDController();

     _CANCoder = new WPI_CANCoder(Constants.MotorConstants.CANCoderID[this.moduleNum], "canivore");

    //  _CANCoder.setPositionToAbsolute(0);
    //  _CANCoder.configAllSettings(returnCANConfig());
     _CANCoder.setPosition(0);
    
     /* Drive Motor Config */
     driveMotor = new CANSparkMax(Constants.MotorConstants.driveMotorIDS[this.moduleNum], MotorType.kBrushless);
     configDriveMotor();

     integratedDriveEncoder = driveMotor.getEncoder();
    //  integratedDriveEncoder.setPositionConversionFactor(1/6.12);
     driveMotor.getPIDController();
     

    pid.enableContinuousInput(0, Math.PI * 2);
    pid.setTolerance(0.0);

    this.invertDriveSpeed = inverted;
    // if(_CANCoder.getMagnetFieldStrength() != MagnetFieldStrength.Good_GreenLED) {
      // throw new RuntimeException("CANCoder on Module #" + Integer.valueOf(this.moduleNum).toString() + " is not green!");
    // }
    m_distFromCenter = distFromCenter;
  }

  public void setSpeedAndAngle(ModuleState targetState) {
    if (Math.abs(currentAngle - targetState.getDir()) > Math.PI/2) { // makes the most optimal turn if angle delta is > Math.PI/2 radians
      setAngle(targetState.getDir() + Math.PI);
      setDriveSpeed(-targetState.getVel());  
    } else {
      setAngle(targetState.getDir());
      setDriveSpeed(targetState.getVel());  
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
    
    currentSpeed = x;
    driveMotor.set(x);
    return x;
  }

  public double getAngleInRadians() { 
    return (_CANCoder.getAbsolutePosition()) * (Math.PI/180.0);
  }

  public double getAngle() {
    return _CANCoder.getPosition();
  }

  public double getDriveVelocity() { //may need to multiply by the gear ratio to get an accurate wheel linear velocity
    return integratedDriveEncoder.getVelocity();
  }

  public double getAngularVelocity() { //may need to multiply by the gear ratio to get an accurate wheel angular velocity
    return integratedAngleEncoder.getVelocity();
  }

  public double getDeltaDist() {
    double newWheelPos = integratedDriveEncoder.getPosition() * (2 * Math.PI) * (1/6.12);
    double ans =  (Constants.wheelDia_m * Math.PI) * (newWheelPos - oldWheelPos);
    oldWheelPos = newWheelPos;
    return ans;
  }

  public void setAngle(double newAngle) {

    currentAngle = newAngle;

    pid.setSetpoint(newAngle);

    String name = "Mod" + String.valueOf(this.moduleNum);
    SmartDashboard.putNumber(name, this.getAngleInRadians());

    if (Math.abs(this.pid.getSetpoint() - this.getAngleInRadians()) > (Constants.MotorConstants.degTolerance*(Math.PI/180))) {
      this.angleMotor.set(MathUtil.clamp(this.pid.calculate(this.getAngleInRadians()), -1, 0.1));
    } else {
      this.angleMotor.set(0.0);
    }

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

  public Translation2d getDistanceFromCenter() {
    return m_distFromCenter;
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
}
