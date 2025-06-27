package org.firstinspires.ftc.teamcode.commands.subsystemcommand.outtakecommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class OuttakeSlideCommand extends InstantCommand {
    public OuttakeSlideCommand (int position) {
        super (
                () -> RobotHardware.getInstance().outtake.setSlideTarget(position)
        );
    }
}
