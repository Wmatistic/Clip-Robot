package org.firstinspires.ftc.teamcode.commands.subsystemcommand.clipmechcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystem.ClipMech;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class ClipMechStateCommand extends InstantCommand {
    public ClipMechStateCommand(ClipMech.ClipMechState clipMechState) {
        super(
                () -> RobotHardware.getInstance().clipMech.setClipMechState(clipMechState)
        );
    }
}
