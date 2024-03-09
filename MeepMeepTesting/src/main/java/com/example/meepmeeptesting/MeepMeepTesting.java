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
        Vector2d startPosition = new Vector2d(51.25, -35.50);
        double startHeading = Math.toRadians(180);

        RoadRunnerBotEntity currentBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(13, 15.19)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(360), Math.toRadians(360), 8.9)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(startPosition, startHeading))
                                .splineTo(new Vector2d(7.00, -59.00), Math.toRadians(180.00))
                                .splineTo(new Vector2d(-36.00, -59.00), Math.toRadians(180.00))
                                .setTangent(Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(-48.00, -35.75, Math.toRadians(180)), Math.toRadians(180.00))
                                .lineToLinearHeading(new Pose2d(-56.75, -35.75, Math.toRadians(180)))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(currentBot)
                .start();
    }
}