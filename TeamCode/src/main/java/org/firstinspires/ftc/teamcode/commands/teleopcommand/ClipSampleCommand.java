package org.firstinspires.ftc.teamcode.commands.teleopcommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.clipmechcommand.ClipLoadedCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.clipmechcommand.ClipMechStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.clipmechcommand.ClipPivotCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.clipmechcommand.IncrementClipCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.clipmechcommand.RailCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.outtakecommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.outtakecommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.outtakecommand.OuttakeStateCommand;
import org.firstinspires.ftc.teamcode.subsystem.ClipMech;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class ClipSampleCommand extends SequentialCommandGroup {
    public ClipSampleCommand() {
        super(
                new ClipMechStateCommand(ClipMech.ClipMechState.CLIPPING),
                new OuttakeClawCommand(Outtake.ClawState.CLOSED),
                new OuttakeArmCommand(RobotConstants.Outtake.armStowed),
                new ClipPivotCommand(RobotConstants.ClipMech.clipPivotUp),
                new RailCommand(RobotHardware.getInstance().clipMech.getClippingPosition()),
                new WaitCommand(400),
                new OuttakeArmCommand(RobotConstants.Outtake.armClip),
                new WaitCommand(300),
                new ClipPivotCommand(RobotConstants.ClipMech.clipPivotDown),
                new WaitCommand(500),
                new OuttakeArmCommand(RobotConstants.Outtake.armClipInter),
                new WaitCommand(600),
                new OuttakeClawCommand(Outtake.ClawState.OPEN),
                new WaitCommand(500),
                new OuttakeClawCommand(Outtake.ClawState.CLOSED),
                new WaitCommand(400),
                new OuttakeArmCommand(RobotConstants.Outtake.armStowed),
                new ClipPivotCommand(RobotConstants.ClipMech.clipPivotPullOut),
                new WaitCommand(400),
                new RailCommand(RobotHardware.getInstance().clipMech.getOutTheWayPosition()),
                new WaitCommand(800),
                new OuttakeArmCommand(RobotConstants.Outtake.armStowed + 0.4),
                new WaitCommand(300),
                new OuttakeArmCommand(RobotConstants.Outtake.armChamberScoreReady),
                new OuttakeStateCommand(Outtake.OuttakeState.SCORING_CHAMBER_INITIAL),
                new ClipLoadedCommand(false),
                new ClipMechStateCommand(ClipMech.ClipMechState.STOWED)
        );
    }
}
