package org.firstinspires.ftc.teamcode.commands.subsystemcommand.clipmechcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class ClipLoadedCommand extends InstantCommand {
    public ClipLoadedCommand(boolean loaded) {
        super(
                () -> Globals.CLIP_LOADED = loaded
        );
    }
}
