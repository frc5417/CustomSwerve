package frc.robot.subsystems;

public class ModuleGroup {
    private final Module[] m_modules;

    public ModuleGroup(Module[] modules) {
        m_modules = modules;
    }

    public void setDrive(ModuleState[] targetModuleStates) {
        if (targetModuleStates.length != m_modules.length)
            throw new IllegalArgumentException("Arguments to ModuleGroup.setDrive() must have the right number of swerve modules!");

        for (int i = 0; i < m_modules.length; i++) {
            m_modules[i].setSpeedAndAngle(targetModuleStates[i]);
        }
    }

    public void resetDrive() {
        for (Module module : m_modules)
            module.resetDriveAngleEncoder();
    }
}
