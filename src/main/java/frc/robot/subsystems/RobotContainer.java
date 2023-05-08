package frc.robot.subsystems;

import frc.robot.Constants;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it u.nder the terms of
// the WPILib BSD license file in the root directory of this project.


import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutonLoader;
import frc.robot.commands.TeleopDrive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
  public static DriveBase driveBase = new DriveBase(kinematics);
  public static AutonLoader autonLoader = new AutonLoader(driveBase);
  public static TeleopDrive teleopDrive = new TeleopDrive(driveBase);
  private final static CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

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
  }


    public static double getLeftJoyX() {
        if (Math.abs(m_driverController.getLeftX()) > Constants.OperatorConstants.joystickDeadband) {
          return -m_driverController.getLeftX();
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
}
