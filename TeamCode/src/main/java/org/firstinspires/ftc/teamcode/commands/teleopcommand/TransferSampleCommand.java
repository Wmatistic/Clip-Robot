package org.firstinspires.ftc.teamcode.commands.teleopcommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.TurretCommand;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.util.RobotConstants;

public class TransferSampleCommand extends SequentialCommandGroup {
    public TransferSampleCommand() {
        super(
                new TurretCommand(RobotConstants.Intake.turretTransfer),
                new WaitCommand(500),
                new ArmCommand(RobotConstants.Intake.armTransfer),
                new WaitCommand(500),
                new ClawCommand(Intake.ClawState.OPEN),
                new WaitCommand(200),
                new ArmCommand(RobotConstants.Intake.armStowed),
                new WaitCommand(500),
                new TurretCommand(RobotConstants.Intake.turretStowed)
        );
    }
}
