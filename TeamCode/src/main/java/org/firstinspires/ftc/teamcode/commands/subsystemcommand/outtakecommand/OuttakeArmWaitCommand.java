package org.firstinspires.ftc.teamcode.commands.subsystemcommand.outtakecommand;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.util.RobotHardware;

import java.util.Collections;
import java.util.Set;

public class OuttakeArmWaitCommand implements Command {
    public OuttakeArmWaitCommand(double position) {
        RobotHardware.getInstance().outtake.setArmTarget(position);
    }

    public boolean isFinished() {
        return RobotHardware.getInstance().outtake.isArmAtTarget();
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Collections.emptySet();
    }
}
