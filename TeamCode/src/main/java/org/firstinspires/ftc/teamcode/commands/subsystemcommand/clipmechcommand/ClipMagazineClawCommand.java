package org.firstinspires.ftc.teamcode.commands.subsystemcommand.clipmechcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystem.ClipMech;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class ClipMagazineClawCommand extends InstantCommand {
    public ClipMagazineClawCommand(ClipMech.ClipHolder clipHolder, double position) {
        super(
                () -> RobotHardware.getInstance().clipMech.setClipHolderMagazineClaws(clipHolder, position)
        );
    }
}
