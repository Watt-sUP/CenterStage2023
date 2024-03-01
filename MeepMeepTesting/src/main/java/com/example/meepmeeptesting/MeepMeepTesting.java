package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
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
                .setConstraints(53, 53, Math.toRadians(240), Math.toRadians(240), 11.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(startPosition, startHeading))
                                .splineTo(new Vector2d(-24.5, -33)
                                        .minus(Vector2d.polar(11, Math.toRadians(30))), Math.toRadians(30.00))
                                .setTangent(Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(-52.50, -35.75, Math.toRadians(180.00)), Math.toRadians(180))
                                .addTemporalMarker(() -> {
                                })
                                .waitSeconds(0.3)
                                .addTemporalMarker(() -> {
                                })
                                .lineToLinearHeading(new Pose2d(-56.50, -35.75, Math.toRadians(180.00)))
                                .addTemporalMarker(() -> {
                                })
                                .waitSeconds(0.5)
                                .setReversed(true)
                                .splineTo(new Vector2d(-24, -11), Math.toRadians(0.00))
                                .splineTo(new Vector2d(24, -11), Math.toRadians(0.00))
                                .splineTo(new Vector2d(50.50, -35.50), Math.toRadians(0.00))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(currentBot)
                .start();
    }
}