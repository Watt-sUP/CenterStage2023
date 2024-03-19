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
        Vector2d startPosition = new Vector2d(16.5, -63.75);
        double startHeading = Math.toRadians(90);

        RoadRunnerBotEntity redBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(13, 15.19)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(270), Math.toRadians(225), 8.9)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(startPosition, startHeading))
                                .splineTo(new Vector2d(23.5, -32).minus(Vector2d.polar(13, Math.toRadians(60))), Math.toRadians(60))
                                .setTangent(0)
                                .splineToSplineHeading(new Pose2d(50.50, -42.50, Math.toRadians(180.00)), Math.toRadians(0))
                                .build()
                );

        RoadRunnerBotEntity blueBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(13, 15.19)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(270), Math.toRadians(225), 8.9)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(mirrorPose(new Pose2d(startPosition, startHeading)))
                                .splineTo(new Vector2d(18, -12), Math.toRadians(180))
                                .waitSeconds(.25)
                                .lineToLinearHeading(new Pose2d(-58.50, -12, Math.toRadians(180)))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(redBot)
                .addEntity(blueBot)
                .start();
    }
}