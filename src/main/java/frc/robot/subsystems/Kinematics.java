// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Kinematics {
  /** Creates a new Compute. */
  private double[] vel = new double[4];
  private double[] theta = new double[4];
  public boolean fieldCentric;
  private double gyro = 0.0;
  private final AHRS m_ahrs;


  public Kinematics(AHRS ahrs) {
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

  public Module.ModuleState[] getComputedModuleStates(ChassisSpeeds targetChassisSpeed) {

    double targetXVelRatio = targetChassisSpeed.vxMetersPerSecond; 
    double targetYVelRatio = targetChassisSpeed.vyMetersPerSecond;
    double targetAngVelRatio = targetChassisSpeed.omegaRadiansPerSecond; 

    if (fieldCentric) {
      this.gyro = this.m_ahrs.getYaw();
      this.gyro *= Math.PI/180;
    } else {
      this.gyro = 0;
    }

    conv(computeUnicorn(computeStrafe(targetXVelRatio, targetYVelRatio), computeRotation(targetAngVelRatio)));

    Module.ModuleState[] targetModuleStates = new Module.ModuleState[4];

    for (int i = 0; i < 4; i++) {
      targetModuleStates[i] = new Module.ModuleState(vel[i], theta[i]);
      // String name = "Swerve (" + String.valueOf(i) + ") Angle";
      // SmartDashboard.putNumber(name, theta[i]);
      String name = "Swerve (" + String.valueOf(i) + ") Speed";
      SmartDashboard.putNumber(name, vel[i]);
    }
    
    return targetModuleStates;
  }

  public double[] getVel() {
  	return vel;
  }
  
  public double[] getTheta() {
  	return theta;
  }
}

