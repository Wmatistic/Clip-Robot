package org.firstinspires.ftc.teamcode.commands.subsystemcommand.outtakecommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class OuttakeClawCommand extends InstantCommand {
    public OuttakeClawCommand(Outtake.ClawState clawState) {
        super(
                () -> RobotHardware.getInstance().outtake.setClawState(clawState)
        );
    }
}
