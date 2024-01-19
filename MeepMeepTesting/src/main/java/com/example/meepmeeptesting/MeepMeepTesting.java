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
                .setConstraints(59.98047895234888, 59.98047895234888, Math.toRadians(234.7068470306528), Math.toRadians(234.7068470306528), 11.08)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(10.85, -64.07, Math.toRadians(90.00)))
                                .splineTo(new Vector2d(14, -40.5), Math.toRadians(90.00))
                                .setReversed(true)
                                .splineTo(new Vector2d(50.50, -35.5), Math.toRadians(0.0))
                                .setReversed(false)
                                .splineTo(new Vector2d(2.12, -61.00), Math.toRadians(180.00))
                                .splineTo(new Vector2d(-24.00, -61.00), Math.toRadians(180.00))
                                .splineTo(new Vector2d(-50.00, -37.50), Math.toRadians(180.00))
                                .splineTo(new Vector2d(-57.00, -37.50), Math.toRadians(180.00))
                                .setReversed(true)
                                .splineTo(new Vector2d(4.00, -13.00), Math.toRadians(0.00))
                                .splineTo(new Vector2d(51.50, -45.00), Math.toRadians(0.00))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}