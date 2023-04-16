package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class DriveBase extends SubsystemBase {

    private static ChassisSpeeds m_chassisSpeeds;
    private final Kinematics m_kinematics;

    public static Module[] moduleGroup;
      

    public DriveBase(Kinematics kinematics) {
        m_kinematics = kinematics;

        moduleGroup = new Module[4];
        for (int i = 0; i < 4; i++)
            moduleGroup[i] = new Module(i, Constants.DriveTrainConstants.invertedMotors[i]);

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

    public void resetDrive() {
        for (int i = 0; i < 4; i++) {
            moduleGroup[i].resetDriveAngleEncoder();
        }
    }

    @Override
    public void periodic() {
    
        Module.ModuleState[] targetModuleStates = m_kinematics.getComputedModuleStates(m_chassisSpeeds);

        for (int i = 0; i < 4; i++)
            moduleGroup[i].setSpeedAndAngle(targetModuleStates[i]);

    }

}