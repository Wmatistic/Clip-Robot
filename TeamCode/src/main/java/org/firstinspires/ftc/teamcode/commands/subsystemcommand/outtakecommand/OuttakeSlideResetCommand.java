package org.firstinspires.ftc.teamcode.commands.subsystemcommand.outtakecommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class OuttakeSlideResetCommand extends InstantCommand {
    public OuttakeSlideResetCommand() {
        super(
                () -> RobotHardware.getInstance().outtake.resetSlides(true)
        );
    }
}
