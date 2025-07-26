package org.firstinspires.ftc.teamcode.commands.teleopcommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.clipmechcommand.ClipPivotCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.clipmechcommand.IncrementClipCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.outtakecommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.outtakecommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.outtakecommand.OuttakeSlideCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.outtakecommand.OuttakeStateCommand;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.util.RobotConstants;

public class ScoreOnChamber extends SequentialCommandGroup {
    public ScoreOnChamber() {
        super(
                new ClipPivotCommand(RobotConstants.ClipMech.clipPivotOutTheWay),
                new WaitCommand(300),
                new OuttakeArmCommand(RobotConstants.Outtake.armChamberScoreReady),
                new OuttakeSlideCommand(RobotConstants.Outtake.slideChamber),
                new WaitCommand(600),
                new OuttakeArmCommand(RobotConstants.Outtake.armChamberScoreInitial),
                new WaitCommand(800),
                new OuttakeSlideCommand(RobotConstants.Outtake.slideChamberScoring),
                new WaitCommand(50),
                new OuttakeArmCommand(RobotConstants.Outtake.armChamberScoreFinal),
                new OuttakeStateCommand(Outtake.OuttakeState.SCORING_CHAMBER_FINAL),
                new IncrementClipCommand()
        );
    }
}
