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
        Vector2d startPosition = new Vector2d(-40.25, -62.75);
        double startHeading = Math.toRadians(90);

        RoadRunnerBotEntity currentBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(13, 15.19)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(65, 65, Math.toRadians(360), Math.toRadians(360), 11.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(startPosition, startHeading))
                                .splineToSplineHeading(new Pose2d(
                                        new Vector2d(-47.5, -33).minus(Vector2d.polar(13, Math.toRadians(180))),
                                        Math.toRadians(180)
                                ), Math.toRadians(90.00))
                                .addTemporalMarker(() -> {
                                })
                                .setTangent(Math.toRadians(90))
                                .splineToLinearHeading(new Pose2d(-48, -16, Math.toRadians(180)), Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(-52.5, -23.5, Math.toRadians(180)), Math.toRadians(-90))
                                .addTemporalMarker(() -> {
                                })
                                .waitSeconds(.3)
                                .addTemporalMarker(() -> {
                                })
                                .lineToLinearHeading(new Pose2d(-56.50, -23.5, Math.toRadians(180.00)))
                                .lineToLinearHeading(new Pose2d(-36, -23.5, Math.toRadians(180)))
                                .setTangent(Math.toRadians(-90))
                                .splineToSplineHeading(new Pose2d(-24, -59, Math.toRadians(180)), Math.toRadians(0))
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
                                .splineTo(mirrorVector(new Vector2d(-39.00, -38.00)), Math.toRadians(-90.00))
                                .setTangent(Math.toRadians(180))
                                .splineToLinearHeading(mirrorPose(new Pose2d(-52.50, -35.75, Math.toRadians(180.00))), Math.toRadians(180))
                                .addTemporalMarker(() -> {
                                })
                                .waitSeconds(0.3)
                                .addTemporalMarker(() -> {
                                })
                                .lineToLinearHeading(mirrorPose(new Pose2d(-56.50, -35.75, Math.toRadians(180.00))))
                                .addTemporalMarker(() -> {
                                })
                                .waitSeconds(0.5)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(currentBot)
                .addEntity(newBot)
                .start();
    }
}