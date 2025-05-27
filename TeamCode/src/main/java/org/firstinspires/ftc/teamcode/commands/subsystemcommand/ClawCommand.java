package org.firstinspires.ftc.teamcode.commands.subsystemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class ClawCommand extends InstantCommand {
    public ClawCommand(Intake.ClawState clawState) {
        super(
                () -> RobotHardware.getInstance().intake.setClawState(clawState)
        );
    }
}
