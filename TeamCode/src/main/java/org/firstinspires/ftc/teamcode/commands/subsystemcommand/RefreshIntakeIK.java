package org.firstinspires.ftc.teamcode.commands.subsystemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.util.IntakeInverseKinematics;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class RefreshIntakeIK extends InstantCommand {
    public RefreshIntakeIK() {
        super(
                () -> IntakeInverseKinematics.calculateIK(RobotHardware.getInstance().limelightClass.getTargetedSample().x, RobotHardware.getInstance().limelightClass.getTargetedSample().y, RobotHardware.getInstance().limelightClass.getTargetedSample().r)
        );
    }
}
