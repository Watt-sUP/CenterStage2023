package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.autonomous.assets.PropLocations;
import org.firstinspires.ftc.teamcode.roadrunner.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.stream.Collectors;

import javax.annotation.Nullable;

public class PathGenerator {

    private final SampleMecanumDrive drive;
    private final Map<PropLocations, Pose2d> purpleLocationsClose = new HashMap<PropLocations, Pose2d>() {{
        put(PropLocations.LEFT, new Pose2d(
                new Vector2d(.5, -33).minus(Vector2d.polar(12, 1.5 * Math.PI)),
                1.5 * Math.PI
        ));
        put(PropLocations.MIDDLE, new Pose2d(15, -38, Math.PI / 2));
        put(PropLocations.RIGHT, new Pose2d(
                new Vector2d(23.5, -32).minus(Vector2d.polar(13, Math.PI / 3)),
                Math.PI / 3
        ));
    }};
    private final Map<PropLocations, Pose2d> purpleLocationsFar = new HashMap<PropLocations, Pose2d>() {{
        put(PropLocations.LEFT, new Pose2d(-46, -39, Math.PI / 2));
        put(PropLocations.MIDDLE, new Pose2d(-39, -38, Math.PI / 2));
        put(PropLocations.RIGHT, new Pose2d(
                new Vector2d(-24.5, -33).minus(Vector2d.polar(11, Math.PI / 6)),
                Math.PI / 6
        ));
    }};

    public PathGenerator(SampleMecanumDrive drive) {
        this.drive = drive;
        assert drive.getRobotLocation() != null :
                "The robot location wasn't set";
    }

    public TrajectorySequence generateStackPath(Pose2d startPose, Stack targetStack) throws IllegalArgumentException {

        if (targetStack == Stack.FAR)
            return drive.trajectorySequenceBuilder(startPose, 60)
                    .splineTo(new Vector2d(17, -11), Math.toRadians(180))
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(52, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                    .lineToLinearHeading(new Pose2d(-56.75, -11.50, Math.toRadians(180)))
                    .build();

        else if (targetStack == Stack.CLOSE) {
            return drive.trajectorySequenceBuilder(startPose, 50)
                    .splineTo(new Vector2d(7.00, -60.00), Math.toRadians(180.00))
                    .splineTo(new Vector2d(-37.00, -60.00), Math.toRadians(180.00))
                    .setConstraints(
                            SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(35)
                    )
                    .setTangent(Math.toRadians(90))
                    .splineToLinearHeading(new Pose2d(-50.00, -35.75, Math.toRadians(180)), Math.toRadians(180.00))
                    .lineToLinearHeading(new Pose2d(-56.75, -35.75, Math.toRadians(180)))
                    .build();
        } else
            throw new IllegalArgumentException("An unexpected error occurred generating a stack trajectory");
    }

    public TrajectorySequence generateBackstagePath(Pose2d startPose, BackstageRoute route) throws IllegalArgumentException {
        return generateBackstagePath(startPose, new Vector2d(51.25, -35.50), route);
    }

    public TrajectorySequence generateBackstagePath(Pose2d startPose, Vector2d endPosition, BackstageRoute route) throws IllegalArgumentException {
        if (route == BackstageRoute.CENTER)
            return drive.trajectorySequenceBuilder(startPose)
                    .setReversed(true)
                    .splineTo(new Vector2d(24, -9), Math.toRadians(0.00))
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(52, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                    .splineTo(endPosition, Math.toRadians(0.00))
                    .build();
        else if (route == BackstageRoute.SIDE)
            return drive.trajectorySequenceBuilder(startPose, 50)
                    .setReversed(true)
                    .splineTo(new Vector2d(-24.00, -58.50), Math.toRadians(0.00))
                    .splineTo(new Vector2d(4.00, -58.50), Math.toRadians(0.00))
                    .splineTo(endPosition, Math.toRadians(0.00))
                    .build();
        else
            throw new IllegalArgumentException("An unexpected error occurred generating a backdrop trajectory");
    }

    @NonNull
    public Map<PropLocations, TrajectorySequence> generatePurpleCases() {
        return Arrays.stream(PropLocations.values())
                .collect(Collectors.toMap(
                        location -> PropLocations.fromId(location.getId() * drive.getRobotLocation().color),
                        location -> {
                            Pose2d purpLoc;

                            if (drive.getRobotLocation().side == 1)
                                purpLoc = purpleLocationsClose.get(location);
                            else purpLoc = purpleLocationsFar.get(location);

                            return drive.trajectorySequenceBuilder(drive.getStartingPosition())
                                    .splineTo(purpLoc.vec(), purpLoc.getHeading())
                                    .build();
                        }
                ));
    }

    @Nullable
    public Map<PropLocations, TrajectorySequence> generateYellowCases() {
        if (drive.getRobotLocation().side == -1)
            return null;

        Map<PropLocations, Pose2d> yellowLocations = new HashMap<PropLocations, Pose2d>() {{
            put(PropLocations.LEFT, new Pose2d(51.25, -29.50, Math.PI));
            put(PropLocations.MIDDLE, new Pose2d(51.25, -35.50, Math.PI));
            put(PropLocations.RIGHT, new Pose2d(51.25, -43.00, Math.PI));
        }};

        return Arrays.stream(PropLocations.values())
                .collect(Collectors.toMap(
                        location -> PropLocations.fromId(location.getId() * drive.getRobotLocation().color),
                        location -> drive.trajectorySequenceBuilder(purpleLocationsClose.get(location))
                                .setTangent(0)
                                .splineToSplineHeading(yellowLocations.get(location), 0)
                                .build()
                ));
    }

    public enum Stack {
        CLOSE, FAR
    }

    public enum BackstageRoute {
        STAGE_DOOR, SIDE_ENTRANCE
    }
}