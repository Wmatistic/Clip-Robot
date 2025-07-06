package org.firstinspires.ftc.teamcode.commands.subsystemcommand.clipmechcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class ClipHolderClawCommand extends InstantCommand {
    public ClipHolderClawCommand(double position) {
        super(
                () -> RobotHardware.getInstance().clipHolderClawServo.setPosition(position)
        );
    }
}
