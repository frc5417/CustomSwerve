// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import org.ejml.simple.SimpleMatrix;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.Matrix;
// import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Kinematics {
  /** Creates a new Compute. */
  private double[] vel = new double[4];
  private double[] theta = new double[4];
  public boolean fieldCentric;
  private double gyro = 0.0;
  private final AHRS m_ahrs;

  private final SimpleMatrix m_forwardKinematics;
  private final SimpleMatrix m_inverseKinematics;

  // private ChassisSpeeds odomSpeeds = new ChassisSpeeds(0, 0, 0);

  private int cnt = 0;


  public Kinematics(AHRS ahrs, Translation2d[] translations) {
    m_inverseKinematics = new SimpleMatrix(8, 3);

    for (int i = 0; i < 4; i++) {
      Translation2d distFromCenter = translations[i];

      m_inverseKinematics.setRow((i * 2), 0, 1, 0, -distFromCenter.getY());
      m_inverseKinematics.setRow((i * 2) + 1, 0, 0, 1, distFromCenter.getX());
    }

    m_forwardKinematics = m_inverseKinematics.pseudoInverse();

    this.fieldCentric = Constants.OperatorConstants.fieldCentric;
    m_ahrs = ahrs;
  }

  private double[][] computeStrafe(double joy_x, double joy_y) {
    double[][] temp_vel = new double[4][2];
    for(int n=0; n<4; n++) {
      temp_vel[n][0] = ((joy_x*Math.cos(gyro)) - (joy_y*Math.sin(gyro)));
      temp_vel[n][1] = ((joy_x*Math.sin(gyro)) + (joy_y*Math.cos(gyro)));
    }
    
    return temp_vel;
  }

  private double[][] computeRotation(double omega) {
    double[][] temp = {{omega * Math.cos(1*(Math.PI/4)), omega * Math.sin(1*(Math.PI/4))},
                       {omega * Math.cos(7*(Math.PI/4)), omega * Math.sin(7*(Math.PI/4))},
                       {omega * Math.cos(3*(Math.PI/4)), omega * Math.sin(3*(Math.PI/4))},
                       {omega * Math.cos(5*(Math.PI/4)), omega * Math.sin(5*(Math.PI/4))}};
    return temp;
  }

  private double[][] addVect(double[][] a, double[][] b) {
    double temp[][] = new double[4][2];
    assert(a.length == b.length);
    for(int n=0; n<a.length; n++) {
      temp[n][0] = a[n][0] + b[n][0];
      temp[n][1] = a[n][1] + b[n][1];
    } 
    return temp;
  }

  public double[][] computeUnicorn(double[][] strafe, double[][] rotation) {
    return this.addVect(strafe, rotation);
  }

  public void conv(double[][] unicorn) {
    for(int i=0; i<unicorn.length; i++) {
      double a = unicorn[i][0];
      double b = unicorn[i][1];
      double combinedVel = Math.sqrt((a*a) + (b*b));

      this.vel[i] = combinedVel;

      if(a == 0) {
        if(b == 0) {
          this.theta[i] = 0;
        } else {
          this.theta[i] = (b/Math.abs(b)) * (Math.PI/2);
        }
      } else {
        this.theta[i] = Math.atan(b/a);
        if (a > 0) {
          if (b >= 0) {
            this.theta[i] += 0;
          } else {
            this.theta[i] += Math.PI*2;
          }
        } else {
          this.theta[i] += Math.PI;
        }
      }
    }
  }

  // public ChassisSpeeds toChassisSpeeds() {
  //   return odomSpeeds;
  // }

  public Twist2d toTwist(Module[] moduleGroup) { // uses forward kinematics to derive a Twist2d from wheel deltas
    //gets the independent x and y velocities of each module based on encoder values

    SimpleMatrix xyVels = new SimpleMatrix(8, 1);
    for (int i = 0; i < 4; i++) {
      double vel = moduleGroup[i].getDeltaDist();
      double dir = moduleGroup[i].getAngleInRadians();

      xyVels.set((i * 2), 0, vel * Math.cos(dir));
      xyVels.set((i * 2) + 1, 0, vel * Math.sin(dir)); 
    }

    //multiply the current x, y velocity matrix by the inverse of the kinematics matrix

    SimpleMatrix finalChassisSpeeds = m_forwardKinematics.mult(xyVels);

    double deltaX = finalChassisSpeeds.get(0, 0);
    double deltaY = finalChassisSpeeds.get(1, 0);
    double deltaOmega = finalChassisSpeeds.get(2, 0);

    return new Twist2d(deltaX, deltaY, deltaOmega);
  }

  public Twist2d toTwistTest(Module.ModuleState[] moduleStates) { // completely for testing purposes
    //gets the independent x and y velocities of each module based on encoder values

    SimpleMatrix xyVels = new SimpleMatrix(8, 1);
    for (int i = 0; i < 4; i++) { 
      double vel = moduleStates[i].getVel() * 0.02 * Constants.Swerve.maxVelocity;
      double dir = moduleStates[i].getDir();

      xyVels.set((i * 2), 0, vel * Math.cos(dir));
      xyVels.set((i * 2) + 1, 0, vel * Math.sin(dir)); 
    }

    //multiply the current x, y velocity matrix by the inverse of the kinematics matrix

    SimpleMatrix finalChassisSpeeds = m_forwardKinematics.mult(xyVels);

    /*if (cnt++ % 50 == 0) {
      System.out.printf("vel: %f, xVel: %f, yVel: %f", targetAngVelRatio, targetXVelRatio, targetYVelRatio);
    }*/

    double deltaX = finalChassisSpeeds.get(0, 0);
    double deltaY = finalChassisSpeeds.get(1, 0);
    double deltaOmega = finalChassisSpeeds.get(2, 0);

    if (Math.abs(deltaX) < 1e-9) {
      deltaX = 0;
    }
    if (Math.abs(deltaY) < 1e-9) {
      deltaY = 0;
    }
    if (Math.abs(deltaOmega) < 1e-9) {
      deltaOmega = 0;
    }

    return new Twist2d(deltaX, deltaY, deltaOmega);
  }

  public Module.ModuleState[] getComputedModuleStates(ChassisSpeeds targetChassisSpeeds) {
      Module.ModuleState[] moduleStates = new Module.ModuleState[4];

      SimpleMatrix velMatrix = new SimpleMatrix(3, 1);
      velMatrix.set(0, 0, targetChassisSpeeds.vxMetersPerSecond);
      velMatrix.set(1, 0, targetChassisSpeeds.vyMetersPerSecond);
      velMatrix.set(2, 0, targetChassisSpeeds.omegaRadiansPerSecond);

      SimpleMatrix results = m_inverseKinematics.mult(velMatrix);

      for (int i = 0; i < 4; i++) {
          double vx = results.get((i * 2), 0);
          double vy = results.get((i * 2 + 1), 0);

          double dir = Math.atan2(vy, vx);
          double vel = Math.sqrt((vx * vx) + (vy * vy));

          moduleStates[i] = new Module.ModuleState(vel, dir);
      }

      return moduleStates;
  }

  // public Module.ModuleState[] getComputedModuleStates(ChassisSpeeds targetChassisSpeed) {

  //   double targetXVelRatio = targetChassisSpeed.vxMetersPerSecond; /// Constants.Swerve.maxVelocity;
  //   double targetYVelRatio = targetChassisSpeed.vyMetersPerSecond; /// Constants.Swerve.maxVelocity;
  //   double targetAngVelRatio = targetChassisSpeed.omegaRadiansPerSecond; /// Constants.Swerve.maxAngularVelocity;

  //   // odomSpeeds = targetChassisSpeed;

  //   if (cnt++ % 50 == 0) {
  //     System.out.printf("vel: %f, xVel: %f, yVel: %f", targetAngVelRatio, targetXVelRatio, targetYVelRatio);
  //   }

  //   if (fieldCentric) {
  //     this.gyro = 0; //this.m_ahrs.getYaw();
  //     this.gyro *= Math.PI/180;
  //   } else {
  //     this.gyro = 0;
  //   }

  //   conv(computeUnicorn(computeStrafe(targetXVelRatio, targetYVelRatio), computeRotation(targetAngVelRatio)));

  //   Module.ModuleState[] targetModuleStates = new Module.ModuleState[4];

  //   for (int i = 0; i < 4; i++) {
  //     targetModuleStates[i] = new Module.ModuleState(vel[i], theta[i]);
  //     String name = "Swerve (" + String.valueOf(i) + ") Angle";
  //     SmartDashboard.putNumber(name, theta[i]);
  //     name = "Swerve (" + String.valueOf(i) + ") Speed";
  //     SmartDashboard.putNumber(name, vel[i]);
  //   }
    
  //   return targetModuleStates;
  // }

  // public static ModuleState[] normalizeVelocity(ModuleState[] moduleStates) { // keeps the ratio of wheel velocities while ensuring that each never goes above 1
  //   assert(moduleStates.length == 4);

  //   ModuleState[] newModuleStates = new ModuleState[4];

  //   double maxVel = 0;
  //   for (int i = 0; i < 4; i++)
  //       Math.max(maxVel, Math.abs(moduleStates[i].getVel()));

  //   if (maxVel > 1) {
  //     for (int i = 0; i < 4; i++) {
  //       newModuleStates[i] = new ModuleState(moduleStates[i].getVel() / maxVel, moduleStates[i].getDir());
  //     }
  //   } else {
  //     newModuleStates = moduleStates;
  //   }

  //   return newModuleStates;
  // } 
}

