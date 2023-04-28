package frc.robot.commands;

import frc.robot.subsystems.DriveBase;
import frc.robot.autons.MoveForward;

// This file will be used to store auton commands (activating intake, etc.)
public class AutonCommands {

    //public static final RotateInPlace ROTATE_IN_PLACE = new RotateInPlace();
    public final DriveBase m_drivebase;
    public final MoveForward MOVE_FORWARD;

    public AutonCommands(DriveBase dBase) {
        m_drivebase = dBase;
        MOVE_FORWARD = new MoveForward(m_drivebase);
    }
}