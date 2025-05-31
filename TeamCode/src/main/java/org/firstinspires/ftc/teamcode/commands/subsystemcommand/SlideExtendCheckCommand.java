package org.firstinspires.ftc.teamcode.commands.subsystemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.util.IntakeInverseKinematics;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class SlideExtendCheckCommand extends InstantCommand {
    public SlideExtendCheckCommand(int extension) {
        super(
                () -> Intake.slideSampleCheck = extension
        );
    }
}
