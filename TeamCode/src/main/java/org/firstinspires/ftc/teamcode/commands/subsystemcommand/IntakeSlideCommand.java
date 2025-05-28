package org.firstinspires.ftc.teamcode.commands.subsystemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class IntakeSlideCommand extends InstantCommand {
    public IntakeSlideCommand(int position) {
        super(
                () -> RobotHardware.getInstance().intake.setExtensionTarget(position)
        );
    }
}
