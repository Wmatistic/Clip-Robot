package org.firstinspires.ftc.teamcode.commands.subsystemcommand.intakecommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class ClawRotationCommand extends InstantCommand {
    public ClawRotationCommand(double position) {
        super(
                () -> RobotHardware.getInstance().clawRotationServo.setPosition(position)
        );
    }
}
