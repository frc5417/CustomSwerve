package frc.robot.subsystems;
import frc.robot.ModuleUtilities.ModuleGroup;
import frc.robot.ModuleUtilities.ModuleState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DriveBase extends SubsystemBase {

    private static ChassisSpeeds m_chassisSpeeds;
    private final ModuleGroup m_moduleGroup;
    private final Kinematics m_kinematics;

    public DriveBase() {
        m_moduleGroup = Systems.moduleGroup; 
        m_kinematics = Systems.kinematics;

        m_moduleGroup.resetDrive();
        m_chassisSpeeds = new ChassisSpeeds(0, 0, 0);
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
    
        ModuleState[] targetModuleStates = m_kinematics.getComputedModuleStates(m_chassisSpeeds);
        // targetModuleStates = Kinematics.normalizeVelocity(targetModuleStates);

        m_moduleGroup.setDrive(targetModuleStates);
    }

}