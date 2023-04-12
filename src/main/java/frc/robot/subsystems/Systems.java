package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutonLoader;
import frc.robot.commands.TeleopDrive;

public class Systems {
    
    private final static Module[] modules = new Module[Constants.DriveTrainConstants.wheels];
      public final ModuleGroup moduleGroup;
    
      public final DriveBase driveBase;
      public final Kinematics kinematics;
      public final AutonLoader autonLoader;
      public final TeleopDrive teleopDrive;
      private final static CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
      public final AHRS ahrs = new AHRS(SerialPort.Port.kMXP);

    public Systems() {
        for (int i = 0; i < Constants.DriveTrainConstants.wheels; i++)
            modules[i] = new Module(i, Constants.DriveTrainConstants.invertedMotors[i]);
        moduleGroup = new ModuleGroup(modules);

        driveBase = new DriveBase(this);
        kinematics = new Kinematics(this);
        autonLoader = new AutonLoader(this);
        teleopDrive = new TeleopDrive(this);
    }

    public static double getLeftJoyX() {
        if (Math.abs(m_driverController.getLeftX()) > Constants.OperatorConstants.joystickDeadband) {
          return -1 * m_driverController.getLeftX();
        } else {
          return 0;
        }
      }
      public static double getLeftJoyY() {
        if (Math.abs(m_driverController.getLeftY()) > Constants.OperatorConstants.joystickDeadband) {
          return m_driverController.getLeftY();
        } else {
          return 0;
        }
      }
      public static double getRightJoyX() {
        if (Math.abs(m_driverController.getRightX()) > Constants.OperatorConstants.joystickDeadband) {
          return m_driverController.getRightX();
        } else {
          return 0;
        }
      }
}
