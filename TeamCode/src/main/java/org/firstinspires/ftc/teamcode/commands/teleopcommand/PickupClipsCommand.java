package org.firstinspires.ftc.teamcode.commands.teleopcommand;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.clipmechcommand.ClipMagazineClawCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.clipmechcommand.ClipMagazineCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.clipmechcommand.ClipMechStateCommand;
import org.firstinspires.ftc.teamcode.subsystem.ClipMech;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

import java.util.HashMap;

public class PickupClipsCommand extends SelectCommand {
    public PickupClipsCommand() {
        super(
                () -> new SelectCommand(
                        new HashMap<Object, Command>() {{
                            put(ClipMech.ClipMechState.STOWED, new SequentialCommandGroup(
                                    new ClipMagazineCommand(ClipMech.ClipHolder.BOTH, RobotConstants.ClipMech.clipMagazinePickupInitial),
                                    new ClipMagazineClawCommand(ClipMech.ClipHolder.BOTH, RobotConstants.ClipMech.clipMagazineClawOpen),
                                    new WaitCommand(500),
                                    new ClipMechStateCommand(ClipMech.ClipMechState.LOAD_MAGAZINE_ONE)
                            ));
                            put(ClipMech.ClipMechState.LOAD_MAGAZINE_ONE, new SequentialCommandGroup(
                                    new ClipMagazineCommand(ClipMech.ClipHolder.BOTH, RobotConstants.ClipMech.clipMagazinePickupTouching),
                                    new ClipMagazineClawCommand(ClipMech.ClipHolder.BOTH, RobotConstants.ClipMech.clipMagazineClawOpen),
                                    new WaitCommand(500),
                                    new ClipMechStateCommand(ClipMech.ClipMechState.LOAD_MAGAZINE_TWO)
                            ));
                            put(ClipMech.ClipMechState.LOAD_MAGAZINE_TWO, new SequentialCommandGroup(
                                    new ClipMagazineClawCommand(ClipMech.ClipHolder.BOTH, RobotConstants.ClipMech.clipMagazineClawClosed),
                                    new WaitCommand(500),
                                    new ClipMagazineCommand(ClipMech.ClipHolder.BOTH, RobotConstants.ClipMech.clipMagazinePickupLifted),
                                    new ClipMechStateCommand(ClipMech.ClipMechState.LOAD_MAGAZINE_THREE)
                            ));
                            put(ClipMech.ClipMechState.LOAD_MAGAZINE_THREE, new SequentialCommandGroup(
                                    new ClipMagazineCommand(ClipMech.ClipHolder.BOTH, RobotConstants.ClipMech.clipMagazineStowed),
                                    new ClipMagazineClawCommand(ClipMech.ClipHolder.BOTH, RobotConstants.ClipMech.clipMagazineClawClosed),
                                    new WaitCommand(400),
                                    new ClipMechStateCommand(ClipMech.ClipMechState.STOWED),
                                    new InstantCommand(() -> Globals.CLIP_MAGAZINES_LOADED = true)
                            ));
                        }},
                        () -> RobotHardware.getInstance().clipMech.getClipMechState()
                )
        );
    }
}
