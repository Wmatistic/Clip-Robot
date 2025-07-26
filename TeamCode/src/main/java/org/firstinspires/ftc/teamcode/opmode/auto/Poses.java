package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

public class Poses {


    // X + is forward, Y + is left
    @Config
    public static class RedSpec {
        public static Pose2d start = new Pose2d(0, 0, Math.toRadians(0));

        // Score Preload
        public static Pose2d chamberPreload = new Pose2d(28, 5, Math.toRadians(0));
        public static Pose2d chamberBackUp = new Pose2d(13, 5, Math.toRadians(0));

        public static Pose2d pickupClipsFar = new Pose2d(10, -38, Math.toRadians(0));
        public static Pose2d pickupClipsClose = new Pose2d(-3, -35, Math.toRadians(0));

        public static Pose2d pickupSpikeMark = new Pose2d(15, -35, Math.toRadians(0));

        public static Pose2d intaking = new Pose2d(27, 3, Math.toRadians(0));
        public static Pose2d intaking2 = new Pose2d(31, 3, Math.toRadians(0));
        public static Pose2d clippingBackUp = new Pose2d(26, intaking.position.y, Math.toRadians(0));
    }
}
