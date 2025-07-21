package org.firstinspires.ftc.teamcode.commands.teleopcommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.clipmechcommand.ClipPivotCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.clipmechcommand.RailCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.outtakecommand.CheckForSampleCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.outtakecommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.outtakecommand.OuttakeArmWaitCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.outtakecommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.outtakecommand.OuttakeSlideCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.outtakecommand.OuttakeSlideResetCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.outtakecommand.OuttakeStateCommand;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class StowOuttakeSlides extends SequentialCommandGroup {
    public StowOuttakeSlides() {
        super(
                new RailCommand(RobotHardware.getInstance().clipMech.getOutTheWayPosition()),
                new ClipPivotCommand(RobotConstants.ClipMech.clipPivotOutTheWay),
                new OuttakeClawCommand(Outtake.ClawState.OPEN),
                new OuttakeSlideCommand(RobotConstants.Outtake.slideStowed),
                new WaitCommand(700),
                new OuttakeSlideResetCommand(),
                new WaitCommand(200),
                new OuttakeClawCommand(Outtake.ClawState.CLOSED),
                new WaitCommand(100),
                new OuttakeArmCommand(RobotConstants.Outtake.armStowed),
                new WaitCommand(600),
                new WaitCommand(400),
                new OuttakeArmCommand(RobotConstants.Outtake.armStowed + 0.2),
                new WaitCommand(300),
                new OuttakeArmCommand(RobotConstants.Outtake.armStowed),
                new OuttakeStateCommand(Outtake.OuttakeState.STOWED),
                new CheckForSampleCommand()
        );
    }
}
