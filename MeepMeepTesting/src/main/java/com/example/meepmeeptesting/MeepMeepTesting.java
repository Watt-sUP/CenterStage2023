package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(640, 60);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(13, 15.19)
                .setStartPose(new Pose2d(10.85, -64.07, Math.toRadians(-90.00)))
                .setConstraints(53, 53, Math.toRadians(180), Math.toRadians(180), 11.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-34.5, -62.75, Math.toRadians(90.00)))
                                .splineTo(new Vector2d(-24.5, -30)
                                        .plus(new Vector2d(0, -13).rotated(Math.toRadians(-30))), Math.toRadians(60.00)).setReversed(true)
                                .splineTo(new Vector2d(-24.00, -60.00), Math.toRadians(0.00))
                                .splineTo(new Vector2d(2.12, -60.00), Math.toRadians(0.00))
                                .splineTo(new Vector2d(50.00, -43.00), Math.toRadians(0.00))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}