// Copyright (c) FIRST and other WPILib contributors. test
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverPort = 0;
    public static final int kManipulatorPort = 1;
    public static final boolean fieldCentric = true; //FRONT IS THE SIDE OPPOSITE TO BATTERY
    public static final double joystickDeadband = 0.1; //HAS TO BE TUNED A BIT
  }

  public static class MotorConstants {
    public static final Integer[] elevatorMotorIDS = {50, 51};
    public static final Boolean[] elevatorMotorInversions = {false, true};

    public static final int wristMotorID = 61;
    public static final boolean wristMotorInversion = false;
    public static final int intakeMotorID = 62;
    public static final boolean intakeMotorInversion = false;

    public static final int throughBoreEncPort = 3;


    // 0 indexing
    public static final Integer[] driveMotorIDS = {11, 21, 31, 40}; 
    public static final Integer[] angleMotorIDS = {10, 20, 30, 41};
    public static final Integer[] CANCoderID = {9, 12, 8, 19};
    public static final Double[] motorDegrees = {90.000, 182.021, 345.146, 339.521};
    public static final Double[] angleOffsets = {0.0, 0.0, 0.0, 0.0};
    public static final Double[][] angleMotorPID = {
      {0.01, 0.0, 0.005}, {0.01, 0.0, 0.005}, {0.01, 0.0, 0.005}, {0.01, 0.0, 0.005}};
    public static final Double degTolerance = 0.75;
  }

  public static class ManipulatorConstants {
    public static final double wristTolerance = 0.05;
    public static final double wristMax = 500.0;
    public static final double wristMin = -500.0;
    // TUNE ALL OF THESE VALUES BASED ON PID, ONE PLAYER SUBSTATION ONLY
    public static final double wristICubeI = 0.0;
    public static final double wristConeICubeO = 0.14;
    public static final double wristConeOCubeI = 0.25;
  }
  
  public static class Swerve {
    public static final Double angularPercentage = 1.0;
    public static final Double XPercentage = -1.0;
    public static final Double YPercentage = - 1.0;

    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;
    public static final double maxVelocity = 3.89; // m/s
    public static final double maxAcceleration = (Double) null; //m/s^2
    public static final double maxAngularVelocity = 12.56637; //rad/sec
    public static final double maxAngularAcceleration = (Double) null; //rad/sec^2
    public static final double maxModuleSpeed = (Double) null;
    public static final double driveBaseRadius = (Double) null; 

    //velocity PID tuning for overall swerve
    public static final double velocitykP = 1.0; // 0.0001
    public static final double velocitykI = 0.0;
    public static final double velocitykD = 0.0;
    public static final double aVelocitykP = 1.0;
    public static final double aVelocitykI = 0.0;
    public static final double aVelocitykD = 0.0;

    // public static final double odomProportionality = -0.93409848871;

    public static final PathConstraints AUTON_CONSTRAINTS = new PathConstraints(maxVelocity, maxAcceleration, maxAngularVelocity, maxAngularAcceleration); // max velocity and acceleration during auton
    public static final long CommandDuration = 1000;
  }

  public static class DriveTrainConstants {
    public static final double DRIVETRAIN_WIDTH = 0.635; // in meters
  
    public static final PIDConstants ROTATION_PID = new PIDConstants(0, 0, 0);
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0, 0, 0);
    public static final Integer wheels = 4;
    public static final boolean[] invertedMotors = {true, true, false, true};
    //TODO: tune pid constants
  }

  public static class Auton {
    public static final String[] paths = {"rotateInPlace, moveForward, PathPlannerTest"};
  }

  // public static class VisionConstants {
  //   public static final Transform3d robotToCam =
  //           new Transform3d(
  //                  new Translation3d(0.5, 0.0, 0.5),
  //                   new Rotation3d(
  //                           0, 0,
  //                           0)); 

  //   // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
  //   public static final String cameraName = "OV5647";
  //   public static final double maxDistanceAway = 2.0;
  //   public static final double forwardKP = 0.1;
  //   public static final double forwardToAngleRatio = 0.5;
    
  //   public static final double CAMERA_HEIGHT_METERS = 0.72;
  //   public static final double TARGET_HEIGHT_METERS = 0;
  //   public static final double CAMERA_PITCH_RADIANS = 0;
  // }

  public static class FieldConstants {
    public static final double length = Units.feetToMeters(54);
    public static final double width = Units.feetToMeters(27);
  } 
}
