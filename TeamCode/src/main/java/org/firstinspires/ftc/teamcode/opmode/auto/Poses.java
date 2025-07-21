package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

public class Poses {


    // X + is forward, Y + is left
    @Config
    public static class RedSpec {
        public static Pose2d start = new Pose2d(0, 0, Math.toRadians(0));

        // Score Preload
        public static Pose2d chamberPreload = new Pose2d(29, 5, Math.toRadians(0));
        public static double chamberBackUp = 10;

        public static Pose2d pickupClipsFar = new Pose2d(10, -35, Math.toRadians(0));
        public static double pickupClipsClose = -2;
    }
}
