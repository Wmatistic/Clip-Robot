package org.firstinspires.ftc.teamcode.commands.subsystemcommand.intakecommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class IntakeStateCommand extends InstantCommand {
    public IntakeStateCommand(Intake.IntakeState intakeState) {
        super(
                () -> RobotHardware.getInstance().intake.setIntakeState(intakeState)
        );
    }
}
