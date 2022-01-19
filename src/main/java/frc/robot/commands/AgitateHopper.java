package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hopper;

public class AgitateHopper extends CommandBase {

    private final Hopper m_hopper;
    private final double m_rpm;

    public AgitateHopper(Hopper hopper, double rpm) {

        m_hopper = hopper;
        m_rpm = rpm;

    }

    @Override
    public void initialize() {
        m_hopper.resetEncoders();
    }

    @Override
    public void execute() {
        m_hopper.agitateHopper(m_rpm);
    }
    
}
