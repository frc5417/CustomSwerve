package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.kauailabs.navx.frc.AHRS;


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

    double mod1Prev = 0;
    double mod1Curr = 0;
    int counter = 0;

    Translation2d m_frontLeftLocation = new Translation2d(-0.323, 0.323);
    Translation2d m_frontRightLocation = new Translation2d(0.323, 0.323);
    Translation2d m_backLeftLocation = new Translation2d(-0.323, -0.323);
    Translation2d m_backRightLocation = new Translation2d(0.323, -0.323);

    SwerveDriveKinematics m_skdKine = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    SwerveDriveOdometry m_sdkOdom;

    Pose2d globalPose = new Pose2d(0.0, 0.0, new Rotation2d());

    public DriveBase(Kinematics kinematics, AHRS ahrs) {
        m_kinematics = kinematics;
        m_ahrs = ahrs;

        moduleGroup = new Module[4];
        for (int i = 0; i < 4; i++) {
            moduleGroup[i] = new Module(i, Constants.DriveTrainConstants.invertedMotors[i]);
            encoderOffset[i] = moduleGroup[i].integratedDriveEncoder.getPosition();
        }

        targetModuleStates = new Module.ModuleState[4];

        for (int i = 0; i < 4; i++)
            targetModuleStates[i] = new Module.ModuleState(0, Constants.MotorConstants.motorDegrees[i] * (Math.PI/180));

        m_sdkOdom = new SwerveDriveOdometry(
            m_skdKine, new Rotation2d(m_ahrs.getAngle()), new SwerveModulePosition[] {
                new SwerveModulePosition(odomDeltas[3], new Rotation2d(odomAngles[3])),
                new SwerveModulePosition(odomDeltas[2], new Rotation2d(odomAngles[2])),
                new SwerveModulePosition(odomDeltas[1], new Rotation2d(odomAngles[1])),
                new SwerveModulePosition(odomDeltas[0], new Rotation2d(odomAngles[0]))
            }, new Pose2d (0.0, 0.0, new Rotation2d())
        );
    } 

    public Pose2d getCurrentPose() {
        return new Pose2d();
        //TODO: FIX THIS
    }

    public void resetOdometry(Pose2d pose) {
        //TODO: FIX THIS
    }

    public void setHardStates(Module.ModuleState[] targetState) {
        targetModuleStates = targetState;
    }

    public void setDriveSpeed(ChassisSpeeds chassisSpeeds) {
        targetModuleStates = m_kinematics.getComputedModuleStates(chassisSpeeds);
    }

    public void resetDrive() {
        for (int i = 0; i < 4; i++) {
            moduleGroup[i].resetDriveAngleEncoder();
        }
    }

    @Override
    public void periodic() {
        // RobotContainer.m_photonsubsystem.updatePose();
        for (int i = 0; i < 4; i++) {
            moduleGroup[i].setSpeedAndAngle(targetModuleStates[i]);
            odomDeltas[i] = (((moduleGroup[i].integratedDriveEncoder.getPosition() - encoderOffset[i])/6.0) * (0.102*2.0*Math.PI)) - 0;
            odomPrevDeltas[i] = odomDeltas[i];
            odomAngles[i] = moduleGroup[i].getAngle();
        }

        globalPose = m_sdkOdom.update(new Rotation2d(m_ahrs.getAngle()), new SwerveModulePosition[] {
            new SwerveModulePosition(odomDeltas[3], new Rotation2d(odomAngles[3])),
            new SwerveModulePosition(odomDeltas[2], new Rotation2d(odomAngles[2])),
            new SwerveModulePosition(odomDeltas[1], new Rotation2d(odomAngles[1])),
            new SwerveModulePosition(odomDeltas[0], new Rotation2d(odomAngles[0]))
        });

        SmartDashboard.putNumber("Mod1_dy/dx", (((moduleGroup[0].integratedDriveEncoder.getPosition() - encoderOffset[0])/6.0) * (0.102*2.0*Math.PI)));
        SmartDashboard.putNumber("Mod2_dy/dx", (((moduleGroup[1].integratedDriveEncoder.getPosition() - encoderOffset[1])/6.0) * (0.102*2.0*Math.PI)));
        SmartDashboard.putNumber("Mod3_dy/dx", (((moduleGroup[2].integratedDriveEncoder.getPosition() - encoderOffset[2])/6.0) * (0.102*2.0*Math.PI)));
        SmartDashboard.putNumber("Mod4_dy/dx", (((moduleGroup[3].integratedDriveEncoder.getPosition() - encoderOffset[3])/6.0) * (0.102*2.0*Math.PI)));

        SmartDashboard.putNumber("GLOBAL POSE X: ", globalPose.getX());
        SmartDashboard.putNumber("GLOBAL POSE Y: ", globalPose.getY());

        // mod1Curr = moduleGroup[0].getAngle();
        // if ((counter++ % 50) == 0) {
        //     System.out.println(odomDeltas[0]);
        // }
        
        // mod1Prev = mod1Curr;
        
    }
}