package org.firstinspires.ftc.teamcode.commands.subsystemcommand.outtakecommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class OuttakeArmCommand extends InstantCommand {
    public OuttakeArmCommand(double position) {
        super(
                () -> RobotHardware.getInstance().outtake.setArmTarget(position)
        );
    }
}
