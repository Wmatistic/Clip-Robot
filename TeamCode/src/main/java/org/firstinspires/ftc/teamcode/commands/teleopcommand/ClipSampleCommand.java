package org.firstinspires.ftc.teamcode.commands.teleopcommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.clipmechcommand.ClipPivotCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.outtakecommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.outtakecommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.outtakecommand.OuttakeSlideCommand;
import org.firstinspires.ftc.teamcode.subsystem.ClipMech;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.util.RobotConstants;

public class ClipSampleCommand extends SequentialCommandGroup {
    public ClipSampleCommand() {
        super(
                new ClipPivotCommand(ClipMech.ClipPivotState.UP),
                new WaitCommand(100),
                new OuttakeClawCommand(Outtake.ClawState.CLOSED),
                new OuttakeArmCommand(RobotConstants.Outtake.armClip),
                new WaitCommand(300),
                new ClipPivotCommand(ClipMech.ClipPivotState.DOWN),
                new WaitCommand(200),
                new OuttakeArmCommand(RobotConstants.Outtake.armClipInter)
        );
    }
}
