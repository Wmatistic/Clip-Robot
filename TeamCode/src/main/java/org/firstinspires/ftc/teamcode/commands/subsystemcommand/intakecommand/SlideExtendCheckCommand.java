package org.firstinspires.ftc.teamcode.commands.subsystemcommand.intakecommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystem.Intake;

public class SlideExtendCheckCommand extends InstantCommand {
    public SlideExtendCheckCommand(int extension) {
        super(
                () -> Intake.slideSampleCheck = extension
        );
    }
}
