package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
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

    private final Map<PropLocations, Pose2d> purpleLocations = new HashMap<PropLocations, Pose2d>() {{
        put(PropLocations.LEFT, new Pose2d(9, -41.5, Math.toRadians(135)));
        put(PropLocations.MIDDLE, new Pose2d(15, -38, Math.toRadians(90)));
        put(PropLocations.RIGHT, new Pose2d(17, -44.25, Math.toRadians(60)));
    }};

    public Map<PropLocations, Vector2d> yellowLocations = new HashMap<PropLocations, Vector2d>() {{
        put(PropLocations.LEFT, new Vector2d(51.25, -29.50));
        put(PropLocations.MIDDLE, new Vector2d(51.25, -35.50));
        put(PropLocations.RIGHT, new Vector2d(51.25, -43.00));
    }};

    public PathGenerator(MecanumDrive drive, AllianceLocation location) {
        this.drive = drive;
        robotLocation = location;
    }

    // TODO: Same thing for the audience side
    public Map<PropLocations, Action> generatePurplePaths() {
        return purpleLocations.entrySet().stream()
                .collect(Collectors.toMap(
                        entry -> PropLocations.fromId(entry.getKey().id * robotLocation.color),
                        entry -> {
                            Pose2d pose = entry.getValue();
                            return drive.actionBuilder(drive.pose, robotLocation)
                                    .splineTo(pose.position, pose.heading)
                                    .build();
                        }
                ));
    }

    // TODO: Same thing for the audience side
    public Map<PropLocations, Action> generateYellowPaths() {
        return Arrays.stream(PropLocations.values())
                .collect(Collectors.toMap(
                        location -> PropLocations.fromId(location.id * robotLocation.color),
                        location -> drive.actionBuilder(purpleLocations.get(location), robotLocation)
                                .setTangent(Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(yellowLocations.get(location), Math.PI),
                                        Math.toRadians(0))
                                .build()
                ));
    }

    public Action generateStackPath(Pose2d beginPose, Stack stackType) {
        if (stackType == Stack.FAR)
            return drive.actionBuilder(beginPose, robotLocation)
                    .splineTo(new Vector2d(17, -12), Math.PI)
                    .splineTo(new Vector2d(-56.75, -11.75), Math.PI,
                            new TranslationalVelConstraint(45.0))
                    .build();
        else if (stackType == Stack.CLOSE)
            return drive.actionBuilder(beginPose, robotLocation)
                    .splineTo(new Vector2d(7, -60), Math.PI)
                    .splineTo(new Vector2d(-37, -60), Math.PI)
                    .setTangent(Math.PI / 2.0)
                    .splineToConstantHeading(new Vector2d(-50, -36), Math.PI)
                    .splineTo(new Vector2d(-56.75, -36), Math.PI,
                            new TranslationalVelConstraint(30))
                    .build();

        throw new RuntimeException("An invalid stack type was chosen during generation");
    }

    public enum Stack {
        CLOSE, FAR
    }
}
