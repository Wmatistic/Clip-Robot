package org.firstinspires.ftc.teamcode.commands.teleopcommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.clipmechcommand.ClipMagazineClawCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.clipmechcommand.ClipMagazineCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.clipmechcommand.ClipLoadedCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.clipmechcommand.ClipMechStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.clipmechcommand.ClipPivotCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.clipmechcommand.IncrementClipCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.clipmechcommand.RailCommand;
import org.firstinspires.ftc.teamcode.subsystem.ClipMech;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class LoadClipCommand extends SequentialCommandGroup {
    public LoadClipCommand() {
        super(
                new RailCommand(RobotHardware.getInstance().clipMech.getClippingPosition()),
                new ClipPivotCommand(RobotConstants.ClipMech.clipPivotTransfer),
                new WaitCommand(500),
                new ClipMagazineCommand(RobotHardware.getInstance().clipMech.getCurrentClipMagazine(), RobotConstants.ClipMech.clipMagazineTransfer),
                new WaitCommand(700),
                new RailCommand(RobotHardware.getInstance().clipMech.getCurrentClipPosition()),
                new WaitCommand(800),
                new ClipMagazineClawCommand(RobotHardware.getInstance().clipMech.getCurrentClipMagazine(), RobotConstants.ClipMech.clipMagazineClawHalfOpen),
                new WaitCommand(500),
                new RailCommand(RobotHardware.getInstance().clipMech.getClippingPosition()),
                new WaitCommand(800),
                new ClipMagazineClawCommand(RobotHardware.getInstance().clipMech.getCurrentClipMagazine(), RobotConstants.ClipMech.clipMagazineClawClosed),
                new ClipPivotCommand(RobotConstants.ClipMech.clipPivotUp),
                new WaitCommand(500),
                new RailCommand(RobotHardware.getInstance().clipMech.getClipSecurePosition()),
                new WaitCommand(200),
                new RailCommand(RobotHardware.getInstance().clipMech.getClippingPosition()),
                new WaitCommand(100),
                new ClipMagazineCommand(RobotHardware.getInstance().clipMech.getCurrentClipMagazine(), RobotConstants.ClipMech.clipMagazineStowed),
                new IncrementClipCommand(),
                new ClipLoadedCommand(true),
                new ClipMechStateCommand(ClipMech.ClipMechState.STOWED)
        );
    }
}
