package frc.robot.commands;

import frc.robot.subsystems.DriveBase;
import frc.robot.Autons.MoveForward;
import frc.robot.Autons.RotateInPlace;

// This file will be used to store auton commands (activating intake, etc.)
public class AutonCommands {

    public final DriveBase m_drivebase;
    public final MoveForward MOVE_FORWARD;
    public final RotateInPlace ROTATE_IN_PLACE;

    public AutonCommands(DriveBase dBase) {
        m_drivebase = dBase;
        MOVE_FORWARD = new MoveForward(m_drivebase);
        ROTATE_IN_PLACE = new RotateInPlace(m_drivebase);
    }
}