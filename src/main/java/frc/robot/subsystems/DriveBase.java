package frc.robot.subsystems;
import com.fasterxml.jackson.databind.module.SimpleAbstractTypeResolver;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class DriveBase extends SubsystemBase {

    private static Module.ModuleState targetModuleStates[];
    private static Module.ModuleState testModuleStates[];
    
    private final Kinematics m_kinematics;
    // private Pose2d currentLocation = new Pose2d();

    public static Module[] moduleGroup;

    // // Locations for the swerve drive modules relative to the robot center.
    // Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
    // Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
    // Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
    // Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

    // // Creating my kinematics object using the module locations
    // SwerveDriveKinematics m_sdskinematics = new SwerveDriveKinematics(
    // m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
    // );

    // SwerveDriveOdometry m_odometry;

    Pose2d m_pose = new Pose2d();
    Pose2d simPose = new Pose2d();

    double prevAngle = 0;
    int cnt = 0;

    public DriveBase(Kinematics kinematics) {

        m_kinematics = kinematics;
        
        moduleGroup = new Module[4];

        testModuleStates = new Module.ModuleState[4];

        for (int i = 0; i < 4; i++)
            moduleGroup[i] = new Module(i, 
            Constants.DriveTrainConstants.invertedMotors[i],
            Constants.DriveTrainConstants.wheelLocations[i]
        );

        for (int i = 0; i < 4; i++)
            testModuleStates[i] = new Module.ModuleState(0, 0);

        // m_odometry = new SwerveDriveOdometry(
        //     m_sdskinematics, RobotContainer.ahrs.getRotation2d(),
        //     new SwerveModulePosition[] {
        //         new SwerveModulePosition(0, new Rotation2d(Constants.MotorConstants.motorDegrees[0])),
        //         new SwerveModulePosition(0, new Rotation2d(Constants.MotorConstants.motorDegrees[1])),
        //         new SwerveModulePosition(0, new Rotation2d(Constants.MotorConstants.motorDegrees[2])),
        //         new SwerveModulePosition(0, new Rotation2d(Constants.MotorConstants.motorDegrees[3]))
        //     }, new Pose2d(5.0, 13.5, new Rotation2d()));

        targetModuleStates = new Module.ModuleState[4];
        
        // for (int i = 0; i < 4; i++)
        //     System.out.printf("initial value for %d is %f\n", i, moduleGroup[i].getAngleInRadians());

        for (int i = 0; i < 4; i++) 
            targetModuleStates[i] = new Module.ModuleState(0, 0);
        
    } 

    public Module[] getModuleGroup() {
        return moduleGroup;
    }

    public Pose2d getCurrentPose() {
        return m_pose;
    }

    // public void resetAngles() {
    //     for (int i = 0; i < 4; i++)
    //         moduleGroup[i].resetAngleOffset();
    // }

    // private void updatePoseManual() { // updates current position of the robot
    //     ChassisSpeeds currentChassisSpeed = m_kinematics.toChassisSpeeds(moduleGroup);
    //     double xVel = currentChassisSpeed.vxMetersPerSecond;
    //     double yVel = currentChassisSpeed.vyMetersPerSecond;
    //     double omega = currentChassisSpeed.omegaRadiansPerSecond;

    //     // TODO: MAKE SURE TO CONFIRM THAT AUTON WORKS WITH DEGREES

    //     currentLocation = new Pose2d(
    //         new Translation2d(
    //             currentLocation.getX() + (xVel * 0.02), 
    //             currentLocation.getY() + (yVel * 0.02)
    //         ),
    //         new Rotation2d(
    //             currentLocation.getRotation().getDegrees() + (omega * (Math.PI / 180.0) * 0.02)
    //         )
    //     );
    // }

    // public void resetOdometry(Pose2d pose) {
    //     SwerveModulePosition[] ModulePoseOdom = {new SwerveModulePosition(0, new Rotation2d(0)),
    //         new SwerveModulePosition(0, new Rotation2d(0)),
    //         new SwerveModulePosition(0, new Rotation2d(0)),
    //         new SwerveModulePosition(0, new Rotation2d(0)),};
    //     for (int i = 0; i < 4; i++) {
    //         ModulePoseOdom[i] = new SwerveModulePosition(targetModuleStates[i].getVel(), new Rotation2d(targetModuleStates[i].getDir()));
    //     }
    //     m_odometry.resetPosition(RobotContainer.ahrs.getRotation2d(), ModulePoseOdom, new Pose2d());
    // }

    public void resetOdometry(Pose2d pose) {
        m_pose = new Pose2d(pose.getTranslation(), RobotContainer.ahrs.getRotation2d());
    }

    private void updateOdom() {

        /*
         * ENSURE that Kinematics::toTwist and getDeltaOmegaAHRS are being called every code cycle
         */

        Twist2d simTwist = m_kinematics.toTwistTest(targetModuleStates);
        Twist2d twisting = m_kinematics.toTwist(moduleGroup);
        //twisting.dtheta = getDeltaOmegaAHRS(); // needs to be tested

        SmartDashboard.putNumber("dX", twisting.dx);
        SmartDashboard.putNumber("dY", twisting.dy);
        SmartDashboard.putNumber("dtheta", twisting.dtheta);

        simPose = simPose.exp(simTwist);
        Pose2d newPose = m_pose.exp(twisting);

        m_pose = new Pose2d(newPose.getTranslation(), RobotContainer.ahrs.getRotation2d());

        SmartDashboard.putNumber("X", m_pose.getX());
        SmartDashboard.putNumber("Y", m_pose.getY());
        SmartDashboard.putNumber("tX", simPose.getX());
        SmartDashboard.putNumber("tY", simPose.getY());

        /*if (cnt++ % 50 == 0) {
            System.out.println(m_pose);
            for (int i = 0; i < 4; i++)
                System.out.printf("m: %d, %f, %f\n", i, moduleGroup[i].getDeltaDist(), moduleGroup[i].getAngleInRadians());
            System.out.println();
        }*/
    }

    public void setDriveSpeed(ChassisSpeeds chassisSpeeds) {
        double xVel = chassisSpeeds.vxMetersPerSecond;
        double yVel = chassisSpeeds.vyMetersPerSecond;
        
        double angle = Math.atan2(yVel, xVel);
        double vel = Math.sqrt((xVel * xVel) + (yVel * yVel));

        if (Constants.OperatorConstants.fieldCentric) {
            angle -= RobotContainer.ahrs.getRotation2d().getRadians();
            //angle -= simPose.getRotation().getRadians(); // modify for when not field centric
        }
        chassisSpeeds = new ChassisSpeeds(vel * Math.cos(angle), vel * Math.sin(angle), chassisSpeeds.omegaRadiansPerSecond);

        targetModuleStates = m_kinematics.getComputedModuleStates(chassisSpeeds);
    }

    public void resetDrive() {
        for (int i = 0; i < 4; i++) {
            moduleGroup[i].resetDriveAngleEncoder();
        }
    }

    private double getDeltaOmegaAHRS() {
        double curAngle = (RobotContainer.ahrs.getAngle() * Math.PI/180);
        double delta = curAngle - prevAngle;
        prevAngle = curAngle;
        return delta;
    }

    @Override
    public void periodic() {
        for (int i = 0; i < 4; i++) {
            moduleGroup[i].setSpeedAndAngle(targetModuleStates[i]);
        }
  
        updateOdom();
    }
}