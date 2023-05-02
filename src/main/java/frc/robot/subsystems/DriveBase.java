package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;


public class DriveBase extends SubsystemBase {

    private static Module.ModuleState targetModuleStates[];
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

    double prevAHRS = 0;
      

    public DriveBase(Kinematics kinematics) {

        m_kinematics = kinematics;

        moduleGroup = new Module[4];
        for (int i = 0; i < 4; i++)
            moduleGroup[i] = new Module(i, 
            Constants.DriveTrainConstants.invertedMotors[i],
            Constants.DriveTrainConstants.wheelLocations[i]
        );

        // m_odometry = new SwerveDriveOdometry(
        //     m_sdskinematics, RobotContainer.ahrs.getRotation2d(),
        //     new SwerveModulePosition[] {
        //         new SwerveModulePosition(0, new Rotation2d(Constants.MotorConstants.motorDegrees[0])),
        //         new SwerveModulePosition(0, new Rotation2d(Constants.MotorConstants.motorDegrees[1])),
        //         new SwerveModulePosition(0, new Rotation2d(Constants.MotorConstants.motorDegrees[2])),
        //         new SwerveModulePosition(0, new Rotation2d(Constants.MotorConstants.motorDegrees[3]))
        //     }, new Pose2d(5.0, 13.5, new Rotation2d()));

        targetModuleStates = new Module.ModuleState[4];

        for (int i = 0; i < 4; i++)
            targetModuleStates[i] = new Module.ModuleState(0, Constants.MotorConstants.motorDegrees[i] * (Math.PI/180));
    } 

    public Pose2d getCurrentPose() {
        return m_pose;
    }

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
        Twist2d twisting = m_kinematics.toTwist(moduleGroup);
        twisting.dtheta = getDeltaOmegaAHRS();
        
        SmartDashboard.putNumber("dX", twisting.dx);
        SmartDashboard.putNumber("dY", twisting.dy);
        SmartDashboard.putNumber("dtheta", twisting.dtheta);

        Pose2d newPose = m_pose.exp(twisting);
        m_pose = new Pose2d(newPose.getTranslation(), RobotContainer.ahrs.getRotation2d());
    }

    public void setDriveSpeed(ChassisSpeeds chassisSpeeds) {
        targetModuleStates = m_kinematics.getComputedModuleStates(chassisSpeeds);
    }

    public void resetDrive() {
        for (int i = 0; i < 4; i++) {
            moduleGroup[i].resetDriveAngleEncoder();
        }
    }

    public double getDeltaOmegaAHRS() {
        double delta = RobotContainer.ahrs.getAngle() - prevAHRS;
        prevAHRS = RobotContainer.ahrs.getAngle();
        return delta;
    }

    @Override
    public void periodic() {
        // SwerveModulePosition[] ModulePoseOdom = {new SwerveModulePosition(0, new Rotation2d(0)),
        //                                          new SwerveModulePosition(0, new Rotation2d(0)),
        //                                          new SwerveModulePosition(0, new Rotation2d(0)),
        //                                          new SwerveModulePosition(0, new Rotation2d(0)),};
        for (int i = 0; i < 4; i++) {
            moduleGroup[i].setSpeedAndAngle(targetModuleStates[i]);
            // ModulePoseOdom[i] = new SwerveModulePosition(targetModuleStates[i].getVel(), new Rotation2d(targetModuleStates[i].getDir()));
        }
            
        // var gyroAngle = RobotContainer.ahrs.getRotation2d();

        // m_pose = m_odometry.update(gyroAngle, ModulePoseOdom);

        updateOdom();

    }
}