package org.firstinspires.ftc.teamcode.commands.teleopcommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.clipmechcommand.ClipPivotCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.clipmechcommand.RailCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.outtakecommand.CheckForSampleCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.outtakecommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.outtakecommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.outtakecommand.OuttakeSlideCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.outtakecommand.OuttakeStateCommand;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.util.RobotConstants;

public class StowOuttakeSlides extends SequentialCommandGroup {
    public StowOuttakeSlides() {
        super(
                new RailCommand(RobotConstants.ClipMech.railOutTheWay),
                new ClipPivotCommand(RobotConstants.ClipMech.clipPivotOutTheWay),
                new OuttakeClawCommand(Outtake.ClawState.OPEN),
                new OuttakeArmCommand(RobotConstants.Outtake.armStowed),
                new WaitCommand(700),
                new OuttakeClawCommand(Outtake.ClawState.CLOSED),
                new OuttakeSlideCommand(RobotConstants.Outtake.slideStowed),
                new WaitCommand(600),
                new OuttakeStateCommand(Outtake.OuttakeState.STOWED),
                new CheckForSampleCommand()
        );
    }
}
