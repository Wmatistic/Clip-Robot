package org.firstinspires.ftc.teamcode.commands.subsystemcommand.intakecommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class RefreshSampleCommand extends InstantCommand {
    public RefreshSampleCommand() {
        super(
                () -> RobotHardware.getInstance().limelightClass.refreshSamples()
        );
    }
}
