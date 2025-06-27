package org.firstinspires.ftc.teamcode.commands.subsystemcommand.intakecommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class TurretCommand extends InstantCommand {
    public TurretCommand(double position) {
        super(
                () -> RobotHardware.getInstance().intake.setTurretTarget(position)
        );
    }
}
