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
  
  private double oldWheelPos = 0.0;

  private final RelativeEncoder integratedDriveEncoder;
  private final RelativeEncoder integratedAngleEncoder;

  private final int moduleNum; // ZERO INDEXED

  private final double kP;
  private final double kI;
  private final double kD;

  public final PIDController pid;

  private int invertDriveSpeed = 1; // global, always applied to drive speed if motor inverted
  private int invertMultiplier = 1; // inverts drive speed if 180 degree code tripped
  private Module.ModuleState currentState;
  private boolean trigger = true;

  private double angleOffset = 0;
  
  private WPI_CANCoder _CANCoder;

  double curDrive = 0;

  private final Translation2d m_distFromCenter;

  int cnt = 0;
  private double deltaDist = 0.0;


  /*
   * 
   * @param the module number 
   * @Param whether the module is inverted or not
   * @param the x and y distance of the module from the robot's center of rotation
   * 
   */

  public Module(int module, boolean inverted, Translation2d distFromCenter) {
    
    this.moduleNum = module;
    this.kP = Constants.MotorConstants.angleMotorPID[module][0];
    this.kI = Constants.MotorConstants.angleMotorPID[module][1];
    this.kD = Constants.MotorConstants.angleMotorPID[module][2];
    this.pid = new PIDController(kP, kI, kD);


    /* Angle Motor Config */
    angleMotor = new CANSparkMax(Constants.MotorConstants.angleMotorIDS[this.moduleNum], MotorType.kBrushless);
    configAngleMotor();
    angleMotor.setSmartCurrentLimit(Constants.MotorConstants.angleMotorCurrentLimit);

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

    // integratedDriveEncoder.setPositionConversionFactor(1/6.12);

    pid.enableContinuousInput(0, Math.PI * 2);
    pid.setTolerance(0.0);

    angleOffset = _CANCoder.getPosition() * Math.PI / 180.0;

    System.out.printf("module %d, %f\n", moduleNum, angleOffset);

    this.invertDriveSpeed = (inverted)? -1 : 1;

    // if(_CANCoder.getMagnetFieldStrength() != MagnetFieldStrength.Good_GreenLED) {
      // throw new RuntimeException("CANCoder on Module #" + Integer.valueOf(this.moduleNum).toString() + " is not green!");
    // }

    currentState = new Module.ModuleState(0, 0);
    m_distFromCenter = distFromCenter;
  }

  public void setSpeedAndAngle(ModuleState targetState) {
    currentState = targetState;
    setAngle(targetState.getDir());
    setDriveSpeed(targetState.getVel() / Constants.Swerve.maxVelocity);
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
    //speed = Math.abs(speed);
    double x = speed * invertDriveSpeed * invertMultiplier;
    driveMotor.set(x);
    return x;
  }

  public double getAngleInRadians() { 
    return ((_CANCoder.getPosition()) * (Math.PI/180.0));
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
    return this.deltaDist;
  }

  public double updateDeltaDist() {
    double newWheelPos = getTotalDist();
    this.deltaDist = newWheelPos - oldWheelPos;
    oldWheelPos = newWheelPos;

    return this.deltaDist;
    // TODO: Make deltas negative with 180
  }

  public double getTotalDist() {
    return integratedDriveEncoder.getPosition() / 6.12 * (Math.PI * Constants.DriveTrainConstants.wheelDia_m);
  }

  public double setAngle(double angle_in_rad) {

    double targetAngle = angle_in_rad;
    pid.setSetpoint(targetAngle); // angles are in TRUE BEARING ( angles are negated )

    return angle_in_rad;
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
    driveMotor.setIdleMode(Constants.Swerve.driveNeutralMode);
    driveMotor.burnFlash();
  }

  public Translation2d getDistanceFromCenter() {
    return m_distFromCenter;
  }

  public Module.ModuleState getCurrentState() {
    return currentState;
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

    public String toString() {
      return String.format("Direction: %f, Velocity: %f", m_dir, m_vel);      
    }

  }
}
