package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

enum AllianceColor {
    RED(1), BLUE(-1);

    private final int multiplier;

    AllianceColor(int multiplier) {
        this.multiplier = multiplier;
    }

    /**
     * <p>Mirrors a pose based on the robot alliance.</p>
     * <p>Assumes red as the default alliance.</p>
     *
     * @param pose The pose corresponding to the red color
     * @return The mirrored pose if the alliance color is blue, the same pose otherwise
     */
    public Pose2d convertPose(Pose2d pose) {
        return new Pose2d(pose.getX(), multiplier * pose.getY(), multiplier * pose.getHeading());
    }

    public Vector2d convertVector(Vector2d vector) {
        return new Vector2d(vector.getX(), multiplier * vector.getY());
    }
}

enum StartingPosition {
    BACKDROP(new Pose2d(12.75, -62.75, Math.toRadians(90.00))), AUDIENCE(new Pose2d(-34.50, -62.75, Math.toRadians(90.00)));

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
}

enum Stack {
    CLOSE, FAR
}

public class PathGenerator {

    private final SampleMecanumDrive drive;
    private AllianceColor allianceColor;
    private StartingPosition startingPosition;


    public PathGenerator(SampleMecanumDrive drive) {
        this.drive = drive;
    }

    public void setStartingLocation(AllianceColor color, StartingPosition position) {
        allianceColor = color;
        startingPosition = position;

        drive.setPoseEstimate(allianceColor.convertPose(startingPosition.getValue()));
    }

    public Pose2d getStartingLocation() {
        return allianceColor.convertPose(startingPosition.getValue());
    }

    public TrajectorySequence generateStackPath(Pose2d startPose, Stack targetStack) throws IllegalArgumentException {
        if (targetStack == Stack.FAR)
            return drive.trajectorySequenceBuilder(startPose)
                    .splineTo(allianceColor.convertVector(new Vector2d(10, -11)), Math.toRadians(180))
                    .splineTo(allianceColor.convertVector(new Vector2d(-24, -11)), Math.toRadians(180))
                    .setConstraints(
                            SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL / 2.0, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(20)
                    )
                    .splineTo(allianceColor.convertVector(new Vector2d(-57.50, -12.5)), Math.toRadians(180))
                    .build();

        else if (targetStack == Stack.CLOSE)
            return drive.trajectorySequenceBuilder(startPose)
                    .splineTo(allianceColor.convertVector(new Vector2d(7, -60)), Math.toRadians(180))
                    .splineTo(allianceColor.convertVector(new Vector2d(-24, -60)), Math.toRadians(180))
                    .setConstraints(
                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(25)
                    )
                    .splineTo(allianceColor.convertVector(new Vector2d(-50.00, -36.50)), Math.toRadians(180.00))
                    .splineTo(allianceColor.convertVector(new Vector2d(-57.50, -36.50)), Math.toRadians(180.00))
                    .build();
        else
            throw new IllegalArgumentException("An unexpected error occurred generating a stack trajectory");
    }
}