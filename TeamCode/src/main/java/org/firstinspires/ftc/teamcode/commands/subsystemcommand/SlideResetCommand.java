package org.firstinspires.ftc.teamcode.commands.subsystemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class SlideResetCommand extends InstantCommand {
    public SlideResetCommand() {
        super(
                () -> RobotHardware.getInstance().intake.resetSlides()
        );
    }
}
