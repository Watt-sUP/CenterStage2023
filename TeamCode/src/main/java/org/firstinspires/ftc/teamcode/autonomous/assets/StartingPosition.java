package org.firstinspires.ftc.teamcode.autonomous.assets;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public enum StartingPosition {
    BACKDROP(new Pose2d(16.5, -63.75, Math.toRadians(90.00))),
    AUDIENCE(new Pose2d(-40.25, -63.75, Math.toRadians(90.00)));

    private final Pose2d pose;

    StartingPosition(Pose2d pose) {
        this.pose = pose;
    }

    /**
     * Returns the starting position of the robot based on the starting side.
     *
     * @return Pose2d object containing the starting position
     */
    public Pose2d getValue() {
        return pose;
    }

    /*
    Home field:
    - Backdrop: new Pose2d(16.75, -62.75, Math.toRadians(90.00))
    - Audience: new Pose2d(-40.25, -62.75, Math.toRadians(90.00))

    TehnoZ field (closer to accurate):
    - Backdrop: new Pose2d(16.5, -63.75, Math.toRadians(90.00))
    - Audience: new Pose2d(-40.25, -63.75, Math.toRadians(90.00))

    Field lengths:
    - Home: 357.5cm
    - TehnoZ: 363cm
     */
}