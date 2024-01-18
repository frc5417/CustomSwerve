package frc.robot;


import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it u.nder the terms of
// the WPILib BSD license file in the root directory of this project.


import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutonLoader;
import frc.robot.commands.SetLightConfig;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Kinematics;
import frc.robot.subsystems.LightsControl;
import frc.robot.subsystems.Manipulator;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...


  public static AHRS ahrs = new AHRS(SerialPort.Port.kMXP);
  public static Kinematics kinematics = new Kinematics(ahrs);
  public static DriveBase driveBase = new DriveBase(kinematics, ahrs);
  public static Manipulator manipulator = new Manipulator();
  public static Elevator elevator = new Elevator();
  public static AutonLoader autonLoader = new AutonLoader(driveBase, manipulator, elevator); //NEEDED SUBSYSTEMS FOR AUTON, ELEVATOR NOT USED
  public static TeleopDrive teleopDrive = new TeleopDrive(driveBase, manipulator, elevator); //ALL SUBSYSTEMS
  private final static CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverPort);
  private final static CommandXboxController m_manipulatorController = new CommandXboxController(OperatorConstants.kManipulatorPort);

  private static final LightsControl m_lightsControl = new LightsControl();
  private static final SetLightConfig lightConfigRed = new SetLightConfig(m_lightsControl, 0);
  private static final SetLightConfig lightConfigBlue = new SetLightConfig(m_lightsControl, 4);
  private static final SetLightConfig lightConfigColor1 = new SetLightConfig(m_lightsControl, 1);
  private static final SetLightConfig lightConfigColor2 = new SetLightConfig(m_lightsControl, 2);

  // public static final PhotonSubsystem m_photonsubsystem = new PhotonSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.povUp().onTrue(lightConfigRed);
    m_driverController.povDown().onTrue(lightConfigBlue);
    m_driverController.povLeft().onTrue(lightConfigColor1);
    m_driverController.povRight().onTrue(lightConfigColor2);
  }

  public static void setDriverRumble(double rumbleVal) {
    m_driverController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, rumbleVal);
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
  // public static void setManipulatorRumble(double rumbleVal) {
  //   m_manipulatorController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, rumbleVal);
  // }

  public static double getDriverLeftJoyX() {
    if (Math.abs(m_driverController.getLeftX()) > Constants.OperatorConstants.joystickDeadband) {
      return m_driverController.getLeftX();
    } else {
      return 0;
    }
  }

  public static double getDriverLeftJoyY() {
    if (Math.abs(m_driverController.getLeftY()) > Constants.OperatorConstants.joystickDeadband) {
      return -m_driverController.getLeftY();
    } else {
      return 0;
    }
  }

  public static double getDriverRightJoyX() {
    if (Math.abs(m_driverController.getRightX()) > Constants.OperatorConstants.joystickDeadband) {
      return m_driverController.getRightX();
    } else {
      return 0;
    }
  }

  public static double getDriverRightJoyY() {
    if (Math.abs(m_driverController.getRightY()) > Constants.OperatorConstants.joystickDeadband) {
      return m_driverController.getRightY();
    } else {
      return 0;
    }
  }
  // =========================================================
  public static double getManipulatorLeftJoyY() {
    if (Math.abs(m_manipulatorController.getLeftY()) > Constants.OperatorConstants.joystickDeadband) {
      return m_manipulatorController.getLeftY();
    } else {
      return 0;
    }
  }

  public static double getManipulatorLeftJoyX() {
    if (Math.abs(m_manipulatorController.getLeftX()) > Constants.OperatorConstants.joystickDeadband) {
      return m_manipulatorController.getLeftX();
    } else {
      return 0;
    }
  }

  public static double getManipulatorRightJoyY() {
    if (Math.abs(m_manipulatorController.getRightY()) > Constants.OperatorConstants.joystickDeadband) {
      return m_manipulatorController.getRightY();
    } else {
      return 0;
    }
  }

  public static double getManipulatorRightJoyX() {
    if (Math.abs(m_manipulatorController.getRightX()) > Constants.OperatorConstants.joystickDeadband) {
      return m_manipulatorController.getRightX();
    } else {
      return 0;
    }
  }

  public static double getManipulatorRightTrigger() {
    if (Math.abs(m_manipulatorController.getRightTriggerAxis()) > Constants.OperatorConstants.joystickDeadband) {
      return m_manipulatorController.getRightTriggerAxis();
    } else {
      return 0;
    }
  }

  public static double getManipulatorLeftTrigger() {
    if (Math.abs(m_manipulatorController.getLeftTriggerAxis()) > Constants.OperatorConstants.joystickDeadband) {
      return m_manipulatorController.getLeftTriggerAxis();
    } else {
      return 0;
    }
  }

  public static Boolean getManipulatorBBool() {
    return m_manipulatorController.b().getAsBoolean();
  }

  public static Boolean getManipulatorABool() {
    return m_manipulatorController.a().getAsBoolean();
  }

  public static Boolean getManipulatorXBool() {
    return m_manipulatorController.x().getAsBoolean();
  }

  public static Boolean getManipulatorYBool() {
    return m_manipulatorController.y().getAsBoolean();
  }

  public static Boolean getManipulatorLeftBumperBool() {
    return m_manipulatorController.leftBumper().getAsBoolean();
  }

  public static Boolean getManipulatorRightBumperBool() {
    return m_manipulatorController.rightBumper().getAsBoolean();
  }

  public static long getFPGATime() {
    return HALUtil.getFPGATime();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autonLoader.getAuton();
  }

  public void runTeleopCommand() {
    teleopDrive.schedule();
  }

  public static void setLEDsOff() {
    m_lightsControl.setLightConfig(3);
  }

  public static void setLEDsOn() {
    m_lightsControl.setLightConfig(0);
  }

  public static double findClockTime(double seconds) {
    double clocktime = (seconds/0.02);
    return clocktime;
  }

  public static ChassisSpeeds getSaturatedSpeeds(double xVel, double yVel, double omega) {
    return new ChassisSpeeds(xVel*Constants.Swerve.XPercentage, yVel*Constants.Swerve.YPercentage, omega*Constants.Swerve.angularPercentage);
  }
}
