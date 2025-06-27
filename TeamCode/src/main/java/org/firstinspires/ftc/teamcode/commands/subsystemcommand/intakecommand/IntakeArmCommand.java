package org.firstinspires.ftc.teamcode.commands.subsystemcommand.intakecommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class IntakeArmCommand extends InstantCommand {
    public IntakeArmCommand(double position) {
        super(
                () -> RobotHardware.getInstance().intakeArmServo.setPosition(position)
        );
    }
}
