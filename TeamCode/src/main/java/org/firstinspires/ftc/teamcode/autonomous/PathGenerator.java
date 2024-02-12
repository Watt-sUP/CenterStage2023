package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.autonomous.assets.AllianceColor;
import org.firstinspires.ftc.teamcode.autonomous.assets.BackstageRoute;
import org.firstinspires.ftc.teamcode.autonomous.assets.PropLocations;
import org.firstinspires.ftc.teamcode.autonomous.assets.Stack;
import org.firstinspires.ftc.teamcode.autonomous.assets.StartingPosition;
import org.firstinspires.ftc.teamcode.roadrunner.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Map;

public class PathGenerator {

    private final SampleMecanumDrive drive;
    private AllianceColor allianceColor;
    private StartingPosition startingPosition;
    private PropLocations propLocation;

    public PathGenerator(SampleMecanumDrive drive) {
        this.drive = drive;
    }

    public void setStartingLocation(AllianceColor color, StartingPosition position) {
        allianceColor = color;
        startingPosition = position;

        drive.setPoseEstimate(allianceColor.convertPose(startingPosition.getValue()));
    }

    /**
     * Gets the starting position of the robot with the given localization.
     *
     * @return Pose2d object containing the starting point of the robot
     */
    public Pose2d getStartingPose() {
        return allianceColor.convertPose(startingPosition.getValue());
    }

    public void setPropLocation(PropLocations location) {
        propLocation = allianceColor.convertPropLocation(location);
    }

    /**
     * <p>Generates a path to one of the alliance's stacks. Made to be used in cycling, start should be at the backdrop.</p>
     * <p>The robot uses the stage door to reach the farther stack, and the side entrance to reach the closer one.</p>
     *
     * @param startPose   The starting position of the trajectory
     * @param targetStack The stack to travel to. Distance is driver-relative (closer and farther from the drivers, respectively)
     * @return TrajectorySequence object containing the route to the stack
     * @throws IllegalArgumentException If the stack argument isn't defined properly
     */
    public TrajectorySequence generateStackPath(Pose2d startPose, Stack targetStack) throws IllegalArgumentException {
        if (targetStack == Stack.FAR)
            return drive.trajectorySequenceBuilder(startPose, 50)
                    .splineTo(allianceColor.convertVector(new Vector2d(17, -11)), Math.toRadians(180))
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                    .lineToLinearHeading(allianceColor.convertPose(new Pose2d(-57.50, -11.50, Math.toRadians(180))))
                    .build();

        else if (targetStack == Stack.CLOSE) {
            if (propLocation == PropLocations.LEFT)
                return drive.trajectorySequenceBuilder(startPose, 50)
                        .splineTo(allianceColor.convertVector(new Vector2d(7.00, -60.00)), Math.toRadians(180.00))
                        .splineTo(allianceColor.convertVector(new Vector2d(-35.00, -60.00)), Math.toRadians(180.00))
                        .setConstraints(
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(20)
                        )
                        .splineToLinearHeading(allianceColor.convertPose(new Pose2d(-50.00, -35.75, Math.toRadians(180))), Math.toRadians(180.00))
                        .waitSeconds(.25)
                        .lineToLinearHeading(allianceColor.convertPose(new Pose2d(-57.50, -35.75, Math.toRadians(180))))
                        .build();
            else
                return drive.trajectorySequenceBuilder(startPose, 40)
                        .splineTo(allianceColor.convertVector(new Vector2d(7.00, -58.00)), Math.toRadians(180.00))
                        .splineTo(allianceColor.convertPose(new Pose2d(-24.00, -58.00, Math.toRadians(180.00))))
                        .splineTo(allianceColor.convertVector(new Vector2d(-50.00, -35.50)), Math.toRadians(180.00))
                        .setConstraints(
                                SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(35)
                        )
                        .splineTo(allianceColor.convertPose(new Pose2d(-57.00, -35.50, Math.toRadians(180.00))))
                        .build();
        } else
            throw new IllegalArgumentException("An unexpected error occurred generating a stack trajectory");
    }

    /**
     * Generates a path to the alliance backdrop. Should generally be used when cycling back from the stack.
     *
     * @param startPose The starting position of the trajectory
     * @param route     The route the robot should take. Options include the side or the stage door.
     * @return TrajectorySequence object containing the route
     * @throws IllegalArgumentException If the route argument isn't defined properly
     */
    public TrajectorySequence generateBackstagePath(Pose2d startPose, BackstageRoute route) throws IllegalArgumentException {
        if (route == BackstageRoute.CENTER)
            return drive.trajectorySequenceBuilder(startPose)
                    .setReversed(true)
                    .splineTo(allianceColor.convertPose(new Pose2d(24, -11, Math.toRadians(0.00))))
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                    .splineTo(allianceColor.convertVector(new Vector2d(50.50, -35.50)), Math.toRadians(0.00))
                    .build();
        else if (route == BackstageRoute.SIDE)
            return drive.trajectorySequenceBuilder(startPose, 45)
                    .setReversed(true)
                    .splineTo(allianceColor.convertVector(new Vector2d(-24.00, -60.00)), Math.toRadians(0.00))
                    .splineTo(allianceColor.convertVector(new Vector2d(2.12, -60.00)), Math.toRadians(0.00))
                    .splineTo(allianceColor.convertVector(new Vector2d(50.50, -32.00)), Math.toRadians(0.00))
                    .build();
        else
            throw new IllegalArgumentException("An unexpected error occurred generating a backdrop trajectory");
    }

    public Map<PropLocations, TrajectorySequence> generatePurpleCases() {
        return null;
    }
}