package org.firstinspires.ftc.teamcode.commands.subsystemcommand.intakecommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class SetSlideExtendCheckCommand extends InstantCommand {
    public SetSlideExtendCheckCommand(int extension) {
        super(
                () -> RobotHardware.getInstance().intake.setSlideSampleCheck(extension)
        );
    }
}
