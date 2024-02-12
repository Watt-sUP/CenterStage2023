package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(640, 60);

        RoadRunnerBotEntity newBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(13, 15.19)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(53, 53, Math.toRadians(180), Math.toRadians(180), 11.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(50.50, -29.50, Math.toRadians(180.00)))
                                .splineTo(new Vector2d(7.00, -60.00), Math.toRadians(180.00))
                                .splineTo(new Vector2d(-35.00, -60.00), Math.toRadians(180.00))
                                .splineToConstantHeading(new Vector2d(-50.00, -35.50), Math.toRadians(180))
                                .splineTo(new Vector2d(-57.00, -35.50), Math.toRadians(180.00))
                                .build()
                );

        RoadRunnerBotEntity currentBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(13, 15.19)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(53, 53, Math.toRadians(180), Math.toRadians(180), 11.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(50.50, 29.50, Math.toRadians(180.00)))
                                .splineTo(new Vector2d(7.00, 60.00), Math.toRadians(180.00))
                                .lineToLinearHeading(new Pose2d(-30.00, 60.00, Math.toRadians(180.00)))
                                .lineToLinearHeading(new Pose2d(-57.00, 35.50, Math.toRadians(180.00)))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(newBot)
                .addEntity(currentBot)
                .start();
    }
}