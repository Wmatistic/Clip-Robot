package org.firstinspires.ftc.teamcode.commands.subsystemcommand.outtakecommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class CheckForSampleCommand extends InstantCommand {
    public CheckForSampleCommand() {
        super(
                () -> RobotHardware.getInstance().outtake.updateSample()
        );
    }
}
