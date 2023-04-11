package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;


public class DriveBase extends SubsystemBase {

    private static ChassisSpeeds m_chassisSpeeds;
    private final Kinematics m_kinematics;

    public DriveBase() {
        RobotContainer.m_moduleGroup.resetDrive();
        m_kinematics = RobotContainer.m_kinematics;
    } 

    public Pose2d getCurrentPose() {
        return new Pose2d();
        //TODO: FIX THIS
    }

    public void resetOdometry(Pose2d pose) {
        //TODO: FIX THIS
    }

    public void setDriveSpeed(ChassisSpeeds chassisSpeeds) {
        m_chassisSpeeds = chassisSpeeds;
    }

    @Override
    public void periodic() {
        double xVel = m_chassisSpeeds.vxMetersPerSecond;
        double yVel = m_chassisSpeeds.vyMetersPerSecond;
        double omega = m_chassisSpeeds.omegaRadiansPerSecond;

        ModuleState[] targetModuleStates = m_kinematics.getTargetSpeeds(new ChassisSpeeds(xVel, yVel, omega));

        targetModuleStates = Kinematics.normalizeVelocity(targetModuleStates);

        RobotContainer.m_moduleGroup.setDrive(targetModuleStates);
    }
}