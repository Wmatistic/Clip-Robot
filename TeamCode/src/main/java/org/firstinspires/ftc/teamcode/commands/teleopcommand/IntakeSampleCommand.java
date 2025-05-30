package org.firstinspires.ftc.teamcode.commands.teleopcommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.ClawRotationCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.IntakeSlideCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.TurretCommand;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.util.IntakeInverseKinematics;
import org.firstinspires.ftc.teamcode.util.RobotConstants;

public class IntakeSampleCommand extends SequentialCommandGroup {
    public IntakeSampleCommand() {
        super(
                new ClawCommand(Intake.ClawState.OPEN),
                new IntakeSlideCommand(IntakeInverseKinematics.slideExtension),
                new TurretCommand(IntakeInverseKinematics.turretAngle),
                new WaitCommand(500),
                new ArmCommand(RobotConstants.Intake.armIntake),
                new ClawRotationCommand(IntakeInverseKinematics.clawRotation),
                new WaitCommand(500),
                new ClawCommand(Intake.ClawState.CLOSED),
                new WaitCommand(200),
                new ArmCommand(RobotConstants.Intake.armStowed),
                new WaitCommand(200),
                new IntakeSlideCommand(RobotConstants.Intake.slideStowed),
                new TurretCommand(RobotConstants.Intake.turretStowed),
                new WaitCommand(500)
        );
    }
}
