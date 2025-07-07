package org.firstinspires.ftc.teamcode.commands.subsystemcommand.clipmechcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class IncrementClipCommand extends InstantCommand {
    public IncrementClipCommand() {
        super(
                () -> RobotHardware.getInstance().clipMech.incrementCurrentClip()
        );
    }
}
