package org.firstinspires.ftc.teamcode.commands.subsystemcommand.clipmechcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class RailCommand extends InstantCommand {
    public RailCommand(double position) {
        super(
                () -> RobotHardware.getInstance().clipMech.setRailTarget(position)
        );
    }
}
