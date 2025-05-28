package org.firstinspires.ftc.teamcode.commands.teleopcommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.IntakeSlideCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.TurretCommand;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.util.IntakeInverseKinematics;
import org.firstinspires.ftc.teamcode.util.RobotConstants;

public class IntakeSampleCommand extends SequentialCommandGroup {
    public IntakeSampleCommand() {
        super(
                new IntakeSlideCommand(IntakeInverseKinematics.slideExtension),
                new TurretCommand(IntakeInverseKinematics.turretAngle),
                new WaitCommand(1000),
                new ArmCommand(RobotConstants.Intake.armIntake),
                new WaitCommand(200),
                new ClawCommand(Intake.ClawState.OPEN)
        );
    }
}
