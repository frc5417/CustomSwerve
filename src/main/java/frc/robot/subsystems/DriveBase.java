package frc.robot.subsystems;
import java.lang.module.ModuleDescriptor.Requires;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;


public class DriveBase extends SubsystemBase {

    private static ChassisSpeeds m_chassisSpeeds;
    private final ModuleGroup m_moduleGroup;
    private final Kinematics m_kinematics;

    public DriveBase(Systems system) {
        m_moduleGroup = system.moduleGroup; 
        m_kinematics = system.kinematics;

        m_moduleGroup.resetDrive();
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
    
        ModuleState[] targetModuleStates = m_kinematics.getModuleStates(m_chassisSpeeds);
        targetModuleStates = Kinematics.normalizeVelocity(targetModuleStates);

        m_moduleGroup.setDrive(targetModuleStates);
    }

}