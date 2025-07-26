package org.firstinspires.ftc.teamcode.commands.teleopcommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.outtakecommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.outtakecommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.util.RobotConstants;

public class RegripSampleCommand extends SequentialCommandGroup {
    public RegripSampleCommand() {
        super(
                new SequentialCommandGroup(
                        new OuttakeClawCommand(Outtake.ClawState.CLOSED),
                        new WaitCommand(200),
                        new OuttakeClawCommand(Outtake.ClawState.OPEN),
                        new OuttakeArmCommand(RobotConstants.Outtake.armStowed + 0.4),
                        new WaitCommand(400),
                        new OuttakeClawCommand(Outtake.ClawState.CLOSED)
                )
        );
    }
}
