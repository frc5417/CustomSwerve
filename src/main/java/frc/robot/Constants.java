// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

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
    public static final int kDriverControllerPort = 0;
    public static final boolean fieldCentric = true;
    public static final double joystickDeadband = 0.2;
  }
  public static class MotorConstants {
    // 0 indexing
    public static final Integer[] driveMotorIDS = {11, 21, 31, 40}; 
    public static final Integer[] angleMotorIDS = {10, 20, 30, 41};
    public static final Integer[] CANCoderID = {9, 12, 8, 13};
    public static final Double[] angleOffsets = {0.0, 0.0, 0.0, 0.0};
    public static final Double[][] angleMotorPID = {
      {0.01, 0.0, 0.005}, {0.01, 0.0, 0.005}, {0.01, 0.0, 0.005}, {0.01, 0.0, 0.005}};
  }
  public static class Swerve {
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;
  }
}
