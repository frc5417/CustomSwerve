// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Compute extends SubsystemBase {
  /** Creates a new Compute. */
  private double[] vel = new double[4];
  private double[] theta = new double[4];
  public boolean fieldCentric = true;
  private double gyro = 0;

  public Compute() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.call(1.0, 0.0, 0.0);
  }

  private double[][] computeStrafe(double joy_x, double joy_y) {
    double[][] temp_vel = new double[4][2];
    for(int n=0; n<4; n++) {
      temp_vel[n][0] = ((joy_x*Math.cos(this.gyro)) + (joy_y*Math.sin(this.gyro)));
      temp_vel[n][1] = ((joy_y*Math.cos(this.gyro)) - (joy_x*Math.sin(this.gyro)));
    }
    return temp_vel;
  }

  private double[][] computeRotation(double omega) {
    double[][] temp = {{omega * Math.cos(Math.PI/4), omega * Math.sin(Math.PI/4)}, 
            {omega * Math.cos(7*(Math.PI/4)), omega * Math.sin(7*(Math.PI/4))}, 
            {omega * Math.cos(5*(Math.PI/4)), omega * Math.sin(5*(Math.PI/4))}, 
            {omega * Math.cos(3*(Math.PI/4)), omega * Math.sin(3*(Math.PI/4))}};
    return temp;
  }

  private double[][] addVect(double[][] a, double[][] b) {
    double temp[][] = new double[4][2];
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
      double b = unicorn[i][0];

      this.vel[i] = Math.sqrt((a*a) + (b*b));
      
      if(a == 0) {
        this.theta[i] = (b/Math.abs(b)) * (Math.PI/2);
      } else {
        this.theta[i] = (a/Math.abs(a)) * Math.atan(b/a);
      }
    }
  }

  public void call(double l_joy_x, double l_joy_y, double r_joy_x) {
    // System.out.printf("Strafe: %f \n", this.computeStrafe(l_joy_x, l_joy_y));
    // System.out.printf("Rotation: %f \n", this.computeRotation(r_joy_x));
    gyro *= Math.PI/180;
    if (fieldCentric) {
      this.gyro = this.gyro;//FIXME GET NAVX YAW
    } else {
      this.gyro = 0;
    }

    this.conv(this.computeUnicorn(this.computeStrafe(l_joy_x, l_joy_y), this.computeRotation(r_joy_x)));
  }
}
