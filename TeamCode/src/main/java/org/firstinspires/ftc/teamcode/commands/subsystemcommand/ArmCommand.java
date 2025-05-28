package org.firstinspires.ftc.teamcode.commands.subsystemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class ArmCommand extends InstantCommand {
    public ArmCommand(double position) {
        super(
                () -> RobotHardware.getInstance().armServo.setPosition(position)
        );
    }
}
