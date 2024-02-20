package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    private static Vector2d mirrorVector(Vector2d vec) {
        return new Vector2d(vec.getX(), -vec.getY());
    }

    private static Pose2d mirrorPose(Pose2d pose) {
        return new Pose2d(mirrorVector(pose.vec()), -pose.getHeading());
    }

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(640, 144);
        Vector2d startPosition = new Vector2d(-56.50, -35.75);
        double startHeading = Math.toRadians(180);

        RoadRunnerBotEntity currentBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(13, 15.19)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(53, 53, Math.toRadians(270), Math.toRadians(270), 11.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(startPosition, startHeading))
                                .setReversed(true)
                                .splineTo(new Vector2d(-24.00, -59.00), Math.toRadians(0.00))
//                                .splineToLinearHeading(new Pose2d(-24.00, -59.00, Math.toRadians(180)), Math.toRadians(0.00))
                                .splineTo(new Vector2d(2.12, -59.00), Math.toRadians(0.00))
                                .splineTo(new Vector2d(50.50, -35.50), Math.toRadians(0.00))
                                .build()
                );

        RoadRunnerBotEntity newBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(13, 15.19)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(53, 53, Math.toRadians(270), Math.toRadians(270), 11.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(mirrorPose(new Pose2d(startPosition, startHeading)))
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(-24.00, 59.00, Math.toRadians(180)), Math.toRadians(0.00))
                                .splineTo(new Vector2d(24.00, 59.00), Math.toRadians(0.00))
                                .splineToLinearHeading(new Pose2d(50.00, 34.00, Math.toRadians(180)), Math.toRadians(0.00))
                                .build()
                );

//        RoadRunnerBotEntity otherCases = new DefaultBotBuilder(meepMeep)
//                .setDimensions(13, 15.19)
//                .setConstraints(53, 53, Math.toRadians(180), Math.toRadians(180), 11.5)
//                .setColorScheme(new ColorSchemeRedDark())
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(50.50, 29.50, Math.toRadians(180.00)))
//                                .splineTo(new Vector2d(7.00, 58.00), Math.toRadians(180.00))
//                                .splineTo(new Vector2d(-24.00, 58.00), Math.toRadians(180.00))
//                                .splineTo(new Vector2d(-50.00, 35.50), Math.toRadians(180.00))
//                                .splineTo(new Vector2d(-57.00, 35.50), Math.toRadians(180.00))
//                                .build()
//                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(currentBot)
                .addEntity(newBot)
//                .addEntity(otherCases)
                .start();
    }
}