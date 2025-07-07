package org.firstinspires.ftc.teamcode.commands.teleopcommand;

import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.clipmechcommand.ClipPivotCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.clipmechcommand.RailCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.intakecommand.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.intakecommand.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.intakecommand.ClawRotationCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.intakecommand.IntakeSlideCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.intakecommand.SlideResetCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.intakecommand.TurretCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.outtakecommand.CheckForSampleCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.outtakecommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.outtakecommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.outtakecommand.OuttakeSlideCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.outtakecommand.OuttakeStateCommand;
import org.firstinspires.ftc.teamcode.subsystem.ClipMech;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.util.RobotConstants;

public class TransferSampleCommand extends SequentialCommandGroup {
    public TransferSampleCommand() {
        super(
                new IntakeSlideCommand(RobotConstants.Intake.slideStowed),
                new SlideResetCommand(),
                new OuttakeStateCommand(Outtake.OuttakeState.TRANSFERRING),
                new OuttakeSlideCommand(RobotConstants.Outtake.slideTransfer),
                new TurretCommand(RobotConstants.Intake.turretTransfer),
                new RailCommand(RobotConstants.ClipMech.railOutTheWay),
                new ClipPivotCommand(RobotConstants.ClipMech.clipPivotTransfer),
                new WaitCommand(500),
                new IntakeArmCommand(RobotConstants.Intake.armTransfer),
                new ClawRotationCommand(RobotConstants.Intake.clawRotationTransfer),
                new CheckForSampleCommand(),
                new WaitCommand(500),
                new IntakeClawCommand(Intake.ClawState.OPEN),
                new WaitCommand(200),
                new IntakeArmCommand(RobotConstants.Intake.armStowed),
                new ClawRotationCommand(RobotConstants.Intake.clawRotationStowed),
                new WaitCommand(500),
                new TurretCommand(RobotConstants.Intake.turretStowed),
                new OuttakeClawCommand(Outtake.ClawState.OPEN),
                new OuttakeArmCommand(RobotConstants.Outtake.armStowed),
                new OuttakeSlideCommand(RobotConstants.Outtake.slideStowed),
                new WaitCommand(300),
                new OuttakeClawCommand(Outtake.ClawState.CLOSED),
                new OuttakeStateCommand(Outtake.OuttakeState.STOWED)
        );
    }
}
