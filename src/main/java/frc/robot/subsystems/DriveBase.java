package frc.robot.subsystems;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DriveBase extends SubsystemBase {

    ChassisSpeeds m_chassisSpeeds;

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
        // use m_chassisSpeeds to drive the driveTrain
    }
}