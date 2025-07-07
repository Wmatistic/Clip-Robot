package org.firstinspires.ftc.teamcode.commands.teleopcommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.outtakecommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.outtakecommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.outtakecommand.OuttakeSlideCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.outtakecommand.OuttakeStateCommand;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.util.RobotConstants;

public class ScoreOnChamber extends SequentialCommandGroup {
    public ScoreOnChamber() {
        super(
                new OuttakeArmCommand(RobotConstants.Outtake.armChamberScoreReady),
                new OuttakeSlideCommand(RobotConstants.Outtake.slideChamber),
                new WaitCommand(500),
                new OuttakeArmCommand(RobotConstants.Outtake.armChamberScoreInitial),
                new WaitCommand(200),
                new OuttakeArmCommand(RobotConstants.Outtake.armChamberScoreFinal),
                new OuttakeStateCommand(Outtake.OuttakeState.SCORING_CHAMBER_FINAL)
        );
    }
}
