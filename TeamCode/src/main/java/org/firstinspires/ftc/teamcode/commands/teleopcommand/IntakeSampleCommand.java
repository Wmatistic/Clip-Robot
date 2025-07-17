package org.firstinspires.ftc.teamcode.commands.teleopcommand;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.intakecommand.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.intakecommand.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.intakecommand.ClawRotationCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.intakecommand.IntakeSlideCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.intakecommand.SetSlideExtendCheckCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.intakecommand.TurretCommand;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.util.IntakeInverseKinematics;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class IntakeSampleCommand extends SequentialCommandGroup {
    public IntakeSampleCommand() {
        super(
                new IntakeClawCommand(Intake.ClawState.OPEN),
                new IntakeSlideCommand(IntakeInverseKinematics.slideExtension + RobotHardware.getInstance().intake.getExtensionTarget()),
                new TurretCommand(IntakeInverseKinematics.turretAngle),
                new WaitCommand(1000),
                new IntakeArmCommand(RobotConstants.Intake.armIntake),
                new ClawRotationCommand(IntakeInverseKinematics.clawRotation),
                new WaitCommand(500),
                new IntakeClawCommand(Intake.ClawState.CLOSED),
                new WaitCommand(200),
                new SetSlideExtendCheckCommand(0),
                new ClawRotationCommand(RobotConstants.Intake.clawRotationStowed),
                new IntakeArmCommand(RobotConstants.Intake.armStowed),
                new WaitCommand(200),
                new IntakeSlideCommand(RobotConstants.Intake.slideStowed),
                new TurretCommand(RobotConstants.Intake.turretStowed),
                new WaitCommand(500)
        );
    }
}
