package org.firstinspires.ftc.teamcode.commands.teleopcommand;

import android.transition.Slide;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.IntakeSlideCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.RefreshIntakeIK;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.RefreshSampleCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.SlideExtendCheckCommand;
import org.firstinspires.ftc.teamcode.util.IntakeInverseKinematics;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class SampleCheckCommand extends SequentialCommandGroup {
    public SampleCheckCommand() {
        super(
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new IntakeSlideCommand(IntakeInverseKinematics.slideExtension - 150),
                                new SlideExtendCheckCommand(IntakeInverseKinematics.slideExtension - 150),
                                new WaitCommand(300),
                                new RefreshSampleCommand(),
                                new ConditionalCommand(
                                        new RefreshIntakeIK(),
                                        new WaitCommand(100),
                                        () -> RobotHardware.getInstance().limelightClass.hasSamples()
                                ),
                                new WaitCommand(500),
                                new IntakeSampleCommand()
                        ),
                        new IntakeSampleCommand(),
                        () -> IntakeInverseKinematics.slideExtension >= 200
                ),
                new SlideExtendCheckCommand(0)
        );
    }
}
