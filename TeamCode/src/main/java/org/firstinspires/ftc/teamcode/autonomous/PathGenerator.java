package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.autonomous.assets.AllianceLocation;
import org.firstinspires.ftc.teamcode.autonomous.assets.PropLocations;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.stream.Collectors;

public class PathGenerator {

    private final MecanumDrive drive;
    private final AllianceLocation robotLocation;

    public PathGenerator(MecanumDrive drive, AllianceLocation location) {
        this.drive = drive;
        robotLocation = location;
    }

    public Map<PropLocations, Action> generatePurplePaths() {
        Map<PropLocations, Pose2d> purpleLocations = new HashMap<PropLocations, Pose2d>() {{
            put(PropLocations.LEFT, new Pose2d(9, -41.5, Math.toRadians(135)));
            put(PropLocations.MIDDLE, new Pose2d(15, -38, Math.toRadians(90)));
            put(PropLocations.RIGHT, new Pose2d(17, -44.25, Math.toRadians(60)));
        }};

        return purpleLocations.entrySet().stream()
                .collect(Collectors.toMap(
                        Map.Entry::getKey,
                        entry -> {
                            Pose2d pose = entry.getValue();
                            return drive.actionBuilder(drive.pose, robotLocation)
                                    .splineTo(pose.position, pose.heading)
                                    .build();
                        }
                ));
    }

    public Map<PropLocations, Action> generateYellowPaths() {
        Map<PropLocations, Pose2d> startingPositions = new HashMap<PropLocations, Pose2d>() {{
            put(PropLocations.LEFT, new Pose2d(9, -41.5, Math.toRadians(135)));
            put(PropLocations.MIDDLE, new Pose2d(15, -38, Math.toRadians(90)));
            put(PropLocations.RIGHT, new Pose2d(17, -44.25, Math.toRadians(60)));
        }};

        Map<PropLocations, Vector2d> yellowLocations = new HashMap<PropLocations, Vector2d>() {{
            put(PropLocations.LEFT, new Vector2d(51.25, -29.50));
            put(PropLocations.MIDDLE, new Vector2d(51.25, -35.50));
            put(PropLocations.RIGHT, new Vector2d(51.25, -43.00));
        }};

        return Arrays.stream(PropLocations.values())
                .collect(Collectors.toMap(
                        location -> location,
                        location -> drive.actionBuilder(startingPositions.get(location), robotLocation)
                                .setTangent(Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(yellowLocations.get(location), Math.toRadians(180)),
                                        Math.toRadians(0))
                                .build()
                ));
    }
}
