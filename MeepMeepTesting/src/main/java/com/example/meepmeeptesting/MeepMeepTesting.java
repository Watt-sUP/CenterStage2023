package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(14.96, 15.19)
                .setStartPose(new Pose2d(10.85, 64.07, Math.toRadians(-90.00)))
                .setConstraints(59.98047895234888, 59.98047895234888, Math.toRadians(234.7068470306528), Math.toRadians(234.7068470306528), 11.08)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(51.50, -35.50, Math.toRadians(180.00)))
                                .splineTo(new Vector2d(2.55, -59.5), Math.toRadians(180.00))
                                .splineTo(new Vector2d(-20, -59.5), Math.toRadians(180.00)).setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, 60, 11.08))
                                .splineTo(new Vector2d(-57.00, -36.5), Math.toRadians(180.00))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}