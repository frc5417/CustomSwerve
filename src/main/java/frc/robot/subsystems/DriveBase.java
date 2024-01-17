package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.Constants;

import org.opencv.core.Mat;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveBase extends SubsystemBase {

    private static Module.ModuleState targetModuleStates[];
    private final Kinematics m_kinematics;
    private final AHRS m_ahrs;

    public static Module[] moduleGroup;

    public static double[] odomDeltas = {0, 0, 0, 0};
    public static double[] odomPrevDeltas = {0, 0, 0, 0};
    public static double[] odomAngles = {0, 0, 0, 0};
    public static double[] encoderOffset = {0, 0, 0, 0};
    public static double[] encoderDriveOffset = {0, 0, 0, 0};

    double mod1Prev = 0;
    double mod1Curr = 0;
    int counter = 0;

    double tic, toc = 0;

    Translation2d m_frontLeftLocation = new Translation2d(-0.23495, 0.23495);
    Translation2d m_frontRightLocation = new Translation2d(0.23495, 0.23495);
    Translation2d m_backLeftLocation = new Translation2d(-0.23495, -0.23495);
    Translation2d m_backRightLocation = new Translation2d(0.23495, -0.23495);

    SwerveDriveKinematics m_skdKine = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    SwerveDriveOdometry m_sdkOdom;


    Pose2d globalPose = new Pose2d(0.0, 0.0, new Rotation2d());
    double X = 0.0;
    double Y = 0.0;
    

    ChassisSpeeds autoSetSpeed = new ChassisSpeeds();

    public DriveBase(Kinematics kinematics, AHRS ahrs) {
        m_kinematics = kinematics;
        m_ahrs = ahrs;

        moduleGroup = new Module[4];
        for (int i = 0; i < 4; i++) {
            moduleGroup[i] = new Module(i, Constants.DriveTrainConstants.invertedMotors[i]);
            encoderOffset[i] = moduleGroup[i].getAngleInRadians();
            encoderDriveOffset[i] = moduleGroup[i].integratedDriveEncoder.getPosition();
        }

        targetModuleStates = new Module.ModuleState[4];

        for (int i = 0; i < 4; i++)
            targetModuleStates[i] = new Module.ModuleState(0, Constants.MotorConstants.motorDegrees[i] * (Math.PI/180));

        m_sdkOdom = new SwerveDriveOdometry(
            m_skdKine, m_ahrs.getRotation2d(), new SwerveModulePosition[] {
                new SwerveModulePosition(odomDeltas[3], new Rotation2d(odomAngles[3])),
                new SwerveModulePosition(odomDeltas[2], new Rotation2d(odomAngles[2])),
                new SwerveModulePosition(odomDeltas[1], new Rotation2d(odomAngles[1])),
                new SwerveModulePosition(odomDeltas[0], new Rotation2d(odomAngles[0]))
            }, new Pose2d (0.0, 0.0, new Rotation2d())
        );


    
    } 

    // public Pose2d getCurrentPose() {
    //     return globalPose;
    // }

    public Pose2d getCurrentPose() {
        return new Pose2d(globalPose.getX(), globalPose.getY(), new Rotation2d(globalPose.getRotation().getRadians()+Math.PI/2));
    }

    public ChassisSpeeds getRelativeChassisSpeeds() {
        return autoSetSpeed;
        // return new ChassisSpeeds(m_ahrs.getVelocityX(), m_ahrs.getVelocityY(), m_ahrs.getRate() * (Math.PI/180.0)); //TODO
    }

    public boolean shouldFlipPath() {
        // if (DriverStation.getAlliance().equals(Alliance.Blue)) {
        //     return Constants.Swerve.shouldFlipAuto;
        // } else if ((DriverStation.getAlliance().equals(Alliance.Red))) {
        //     return !Constants.Swerve.shouldFlipAuto;
        // } else {
        //     System.out.println("IDK THE ROBOT ALLIANCE BRO"); 
        //     return false;
        // }
        
        return Constants.Swerve.shouldFlipAuto;
    }

    public void resetOdometry(Pose2d pose) {
        globalPose = pose;
    }

    public void setHardStates(Module.ModuleState[] targetState) {
        targetModuleStates = targetState;
    }

    public void setDriveSpeed(ChassisSpeeds chassisSpeeds) {
        targetModuleStates = m_kinematics.getComputedModuleStates(chassisSpeeds);
    }

    public void setAutoSpeed(ChassisSpeeds chassisSpeeds) {
        autoSetSpeed = chassisSpeeds;
        ChassisSpeeds computed = new ChassisSpeeds(chassisSpeeds.vxMetersPerSecond/Constants.Swerve.maxVelocity, chassisSpeeds.vyMetersPerSecond/Constants.Swerve.maxVelocity, 0.0); //chassisSpeeds.omegaRadiansPerSecond/Constants.Swerve.maxAngularVelocity
        targetModuleStates = m_kinematics.getComputedModuleStates(computed);
    }

    public void resetDrive() {
        for (int i = 0; i < 4; i++) {
            moduleGroup[i].resetDriveAngleEncoder();
        }
    }

    public double smallestAngle(double largeAngle) {
        if(largeAngle > 0) {
            return largeAngle - Math.floor(Math.abs(largeAngle)/(2*Math.PI)) * (2*Math.PI);
        } else {
            return (largeAngle + Math.floor(Math.abs(largeAngle)/(2*Math.PI)) * (2*Math.PI)) + (2*Math.PI);
        }
    }

    @Override
    public void periodic() {
        // RobotContainer.m_photonsubsystem.updatePose();
        for (int i = 0; i < 4; i++) {
            moduleGroup[i].setSpeedAndAngle(targetModuleStates[i]);
            odomDeltas[i] = (((moduleGroup[i].integratedDriveEncoder.getPosition() - encoderDriveOffset[i])/6.12) * (0.102*Math.PI));// - odomPrevDeltas[i];
            odomAngles[i] = smallestAngle(moduleGroup[i].getAngleInRadians());//smallestAngle(moduleGroup[i].getAngleInRadians()*(180.0/Math.PI)) * (Math.PI/180.0);
        }
        

        globalPose = m_sdkOdom.update(m_ahrs.getRotation2d(), new SwerveModulePosition[] {
            new SwerveModulePosition(Math.abs(odomDeltas[3]), new Rotation2d(odomAngles[3])),
            new SwerveModulePosition(Math.abs(odomDeltas[2]), new Rotation2d(odomAngles[2])),
            new SwerveModulePosition(Math.abs(odomDeltas[1]), new Rotation2d(odomAngles[1])),
            new SwerveModulePosition(Math.abs(odomDeltas[0]), new Rotation2d(odomAngles[0]))
        });


        SmartDashboard.putNumber("Yaw", m_ahrs.getYaw());
        SmartDashboard.putNumber("Pitch", m_ahrs.getPitch());
        SmartDashboard.putNumber("Roll", m_ahrs.getRoll());
        SmartDashboard.putNumber("Rotations", m_ahrs.getAngle()/360);

        // double voltage = m_pdp.getVoltage();
        // SmartDashboard.putNumber("Voltage", voltage);

        // X += globalPose.getX();
        // Y += globalPose.getY();

        // SmartDashboard.putNumber("Mod1_delta", Math.abs(odomDeltas[0]));
        // SmartDashboard.putNumber("Mod2_delta", Math.abs(odomDeltas[1]));
        // SmartDashboard.putNumber("Mod3_delta", Math.abs(odomDeltas[2]));
        // SmartDashboard.putNumber("Mod4_delta", Math.abs(odomDeltas[3]));

        SmartDashboard.putNumber("Mod1_theta", -Math.abs(Math.toDegrees(odomAngles[0]))-90);
        SmartDashboard.putNumber("Mod2_theta", -Math.abs(Math.toDegrees(odomAngles[1]))-90);
        SmartDashboard.putNumber("Mod3_theta", -Math.abs(Math.toDegrees(odomAngles[2]))-90);
        SmartDashboard.putNumber("Mod4_theta", -Math.abs(Math.toDegrees(odomAngles[3]))-90);
        
        SmartDashboard.putNumber("GLOBAL POSE X: ", -globalPose.getX());
        SmartDashboard.putNumber("GLOBAL POSE Y: ", -globalPose.getY());

        SmartDashboard.putNumber("Distance Travelled", Math.sqrt((globalPose.getX()*globalPose.getX())+(globalPose.getY()*globalPose.getY())));

        
    }
}
