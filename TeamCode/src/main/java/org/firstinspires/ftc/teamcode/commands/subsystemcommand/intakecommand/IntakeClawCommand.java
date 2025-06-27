package org.firstinspires.ftc.teamcode.commands.subsystemcommand.intakecommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class IntakeClawCommand extends InstantCommand {
    public IntakeClawCommand(Intake.ClawState clawState) {
        super(
                () -> RobotHardware.getInstance().intake.setClawState(clawState)
        );
    }
}
