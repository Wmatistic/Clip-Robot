package org.firstinspires.ftc.teamcode.commands.subsystemcommand.outtakecommand;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.util.RobotHardware;

import java.util.Collections;
import java.util.Set;

public class OuttakeArmCommand extends InstantCommand {
    public OuttakeArmCommand(double position) {
        super(
                () -> RobotHardware.getInstance().outtake.setArmTarget(position)
        );
    }
}
