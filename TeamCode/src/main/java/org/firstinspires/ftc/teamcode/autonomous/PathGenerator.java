package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.autonomous.assets.AllianceColor;
import org.firstinspires.ftc.teamcode.autonomous.assets.BackstageRoute;
import org.firstinspires.ftc.teamcode.autonomous.assets.Stack;
import org.firstinspires.ftc.teamcode.autonomous.assets.StartingPosition;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

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

    public Pose2d getStartingPose() {
        return allianceColor.convertPose(startingPosition.getValue());
    }

    public TrajectorySequence generateStackPath(Pose2d startPose, Stack targetStack) throws IllegalArgumentException {
        if (targetStack == Stack.FAR)
            return drive.trajectorySequenceBuilder(startPose)
                    .splineTo(allianceColor.convertVector(new Vector2d(10.00, -11.00)), Math.toRadians(180.00))
                    .setConstraints(
                            SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(30)
                    )
                    .splineTo(allianceColor.convertVector(new Vector2d(-24.00, -11.00)), Math.toRadians(180.00))
                    .splineTo(allianceColor.convertVector(new Vector2d(-57.50, -12.00)), Math.toRadians(180.00))
                    .build();

        else if (targetStack == Stack.CLOSE)
            return drive.trajectorySequenceBuilder(startPose)
                    .splineTo(allianceColor.convertVector(new Vector2d(7.00, -60.00)), Math.toRadians(180.00))
                    .splineTo(allianceColor.convertVector(new Vector2d(-24.00, -60.00)), Math.toRadians(180.00))
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

    public TrajectorySequence generateBackstagePath(Pose2d startPose, BackstageRoute route) throws IllegalArgumentException {
        if (route == BackstageRoute.CENTER)
            return drive.trajectorySequenceBuilder(startPose, 40)
                    .setReversed(true)
                    .splineTo(allianceColor.convertVector(new Vector2d(4.00, -13.00)), Math.toRadians(0.00))
                    .splineTo(allianceColor.convertVector(new Vector2d(50.00, -42.50)), Math.toRadians(0.00))
                    .build();
        else if (route == BackstageRoute.SIDE)
            return drive.trajectorySequenceBuilder(startPose, 40)
                    .setReversed(true)
                    .splineTo(allianceColor.convertVector(new Vector2d(-24.00, -60.00)), Math.toRadians(0.00))
                    .splineTo(allianceColor.convertVector(new Vector2d(2.12, -60.00)), Math.toRadians(0.00))
                    .splineTo(allianceColor.convertVector(new Vector2d(50.00, -32.00)), Math.toRadians(0.00))
                    .build();
        else
            throw new IllegalArgumentException("An unexpected error occurred generating a backdrop trajectory");
    }
}