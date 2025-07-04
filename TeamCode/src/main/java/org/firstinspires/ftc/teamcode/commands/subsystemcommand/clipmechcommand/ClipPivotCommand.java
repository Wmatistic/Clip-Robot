package org.firstinspires.ftc.teamcode.commands.subsystemcommand.clipmechcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystem.ClipMech;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class ClipPivotCommand extends InstantCommand {
    public ClipPivotCommand(ClipMech.ClipPivotState clipPivotState) {
        super(
                () -> RobotHardware.getInstance().clipMech.setClipPivotState(clipPivotState)
        );
    }
}
