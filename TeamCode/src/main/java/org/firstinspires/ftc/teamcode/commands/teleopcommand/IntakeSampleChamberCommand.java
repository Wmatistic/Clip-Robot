package org.firstinspires.ftc.teamcode.commands.teleopcommand;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.intakecommand.ClawRotationCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.intakecommand.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.intakecommand.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.intakecommand.IntakeSlideCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.intakecommand.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.intakecommand.SetSlideExtendCheckCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.intakecommand.TurretCommand;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.util.IntakeInverseKinematics;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

import java.util.HashMap;
import java.util.Map;

public class IntakeSampleChamberCommand extends SelectCommand {
    public IntakeSampleChamberCommand() {
        super(
                () -> new SelectCommand(
                        new HashMap<Object, Command>() {{
                            put(Intake.IntakeState.INTAKING_CHAMBER_1, new SequentialCommandGroup(
                                    new TurretCommand(RobotConstants.Intake.turretChamber),
                                    new WaitCommand(400),
                                    new ClawRotationCommand(RobotConstants.Intake.clawRotationChamber),
                                    new IntakeArmCommand(RobotConstants.Intake.armChamber),
                                    new IntakeStateCommand(Intake.IntakeState.INTAKING_CHAMBER_2)
                            ));
                            put(Intake.IntakeState.INTAKING_CHAMBER_2, new SequentialCommandGroup(
                                    new IntakeClawCommand(Intake.ClawState.OPEN),
                                    new IntakeSlideCommand(IntakeInverseKinematics.slideExtension),
                                    new TurretCommand(IntakeInverseKinematics.turretAngle),
                                    new ClawRotationCommand(IntakeInverseKinematics.clawRotation),
                                    new WaitCommand(1500),
                                    new IntakeArmCommand(RobotConstants.Intake.armIntake),
                                    new WaitCommand(500),
                                    new IntakeClawCommand(Intake.ClawState.CLOSED),
                                    new WaitCommand(200),
                                    new IntakeArmCommand(RobotConstants.Intake.armChamber),
                                    new ClawRotationCommand(RobotConstants.Intake.clawRotationChamber),
                                    new WaitCommand(200),
                                    new TurretCommand(RobotConstants.Intake.turretChamber),
                                    new WaitCommand(400),
                                    new SetSlideExtendCheckCommand(0),
                                    new IntakeSlideCommand(RobotConstants.Intake.slideStowed),
                                    new ConditionalCommand(
                                            new IntakeStateCommand(Intake.IntakeState.INTAKING_CHAMBER_3),
                                            new IntakeStateCommand(Intake.IntakeState.INTAKING_CHAMBER_2),
                                            () -> RobotHardware.getInstance().intake.isSample()
                                    )
                            ));
                            put(Intake.IntakeState.INTAKING_CHAMBER_3, new SequentialCommandGroup(
                                    new IntakeArmCommand(RobotConstants.Intake.armStowed),
                                    new WaitCommand(50),
                                    new ClawRotationCommand(RobotConstants.Intake.clawRotationStowed),
                                    new WaitCommand(300),
                                    new TurretCommand(RobotConstants.Intake.turretStowed),
                                    new IntakeStateCommand(Intake.IntakeState.STOWED)
                            ));
                        }},
                        () -> RobotHardware.getInstance().intake.getIntakeState()
                )
        );
    }
}
