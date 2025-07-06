package org.firstinspires.ftc.teamcode.commands.subsystemcommand.clipmechcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystem.ClipMech;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class ClipPivotCommand extends InstantCommand {
    public ClipPivotCommand(double position) {
        super(
                () -> RobotHardware.getInstance().clipPivotServo.setPosition(position)
        );
    }
}
