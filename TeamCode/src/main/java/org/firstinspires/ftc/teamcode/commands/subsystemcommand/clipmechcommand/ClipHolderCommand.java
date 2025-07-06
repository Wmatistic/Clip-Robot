package org.firstinspires.ftc.teamcode.commands.subsystemcommand.clipmechcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class ClipHolderCommand extends InstantCommand {
    public ClipHolderCommand(double position) {
        super(
                () -> RobotHardware.getInstance().clipMech.setClipHolderTarget(position)
        );
    }
}
