package frc.robot.subsystems;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;


public class DriveBase extends SubsystemBase {

    private static ChassisSpeeds m_chassisSpeeds;
    private final Module[] m_modules;
    private final Compute compute;

    public DriveBase() {
        m_modules = RobotContainer.m_modules;
        compute = RobotContainer.m_Compute;

        for (int i = 0; i < 4; i++) {
            m_modules[i].setAngle(0);
            m_modules[i].setDriveSpeed(0);
        }
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

        double xVelRatio = xVel / Constants.Swerve.maxVelocity;
        double yVelRatio = yVel / Constants.Swerve.maxVelocity;
        double omegaRatio = omega / Constants.Swerve.maxAngularVelocity;
        
        compute.update(xVelRatio, yVelRatio, omegaRatio);
        double[] vels = compute.getVel();
        double[] dirs = compute.getTheta();
        
        Compute.normalizeVelocity(vels);

        assert(vels.length == 4 && dirs.length == 4);

        for (int i = 0; i < 4; i++) {
            m_modules[i].setAngle(dirs[i]);
            m_modules[i].setDriveSpeed(vels[i]);
        }
    }
}