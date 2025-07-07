package org.firstinspires.ftc.teamcode.commands.subsystemcommand.outtakecommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class OuttakeStateCommand extends InstantCommand {
    public OuttakeStateCommand(Outtake.OuttakeState outtakeState) {
        super(
                () -> RobotHardware.getInstance().outtake.setOuttakeState(outtakeState)
        );
    }
}
