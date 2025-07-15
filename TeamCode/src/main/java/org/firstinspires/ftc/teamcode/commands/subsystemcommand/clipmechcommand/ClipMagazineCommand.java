package org.firstinspires.ftc.teamcode.commands.subsystemcommand.clipmechcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystem.ClipMech;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class ClipMagazineCommand extends InstantCommand {
    public ClipMagazineCommand(ClipMech.ClipHolder clipHolder, double position) {
        super(
                () -> RobotHardware.getInstance().clipMech.setClipHolderMagazines(clipHolder, position)
        );
    }
}
