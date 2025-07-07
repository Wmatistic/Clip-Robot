package org.firstinspires.ftc.teamcode.commands.teleopcommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.clipmechcommand.ClipHolderClawCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.clipmechcommand.ClipHolderCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.clipmechcommand.ClipLoadedCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.clipmechcommand.ClipPivotCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.clipmechcommand.IncrementClipCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.clipmechcommand.RailCommand;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class LoadClipCommand extends SequentialCommandGroup {
    public LoadClipCommand() {
        super(
                new RailCommand(RobotConstants.ClipMech.railStowed),
                new ClipPivotCommand(RobotConstants.ClipMech.clipPivotTransfer),
                new WaitCommand(500),
                new ClipHolderCommand(RobotConstants.ClipMech.clipHolderTransfer),
                new WaitCommand(500),
                new RailCommand(RobotHardware.getInstance().clipMech.getCurrentClipPosition()),
                new WaitCommand(500),
                new ClipHolderClawCommand(RobotConstants.ClipMech.clipHolderClawHalfOpen),
                new WaitCommand(500),
                new RailCommand(RobotConstants.ClipMech.railClipping),
                new WaitCommand(500),
                new ClipHolderClawCommand(RobotConstants.ClipMech.clipHolderClawClosed),
                new ClipPivotCommand(RobotConstants.ClipMech.clipPivotUp),
                new WaitCommand(500),
                new RailCommand(RobotConstants.ClipMech.railSecureClip),
                new WaitCommand(200),
                new RailCommand(RobotConstants.ClipMech.railClipping),
                new WaitCommand(100),
                new ClipHolderCommand(RobotConstants.ClipMech.clipHolderStowed),
                new IncrementClipCommand(),
                new ClipLoadedCommand(true)
        );
    }
}
