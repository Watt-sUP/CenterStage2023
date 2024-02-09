package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.security.InvalidParameterException;
import java.util.Locale;

/**
 * Creates a command that runs a RoadRunner path based on a certain case.
 * <p>Cases included: LEFT, MIDDLE, RIGHT (case insensitive)</p>
 */
public class RunByCaseCommand extends CommandBase {
    private final SampleMecanumDrive drive;
    private final Object left, middle, right;
    private final String targetCase;
    private final boolean blocking;

    /**
     * Creates a new RunByCaseCommand instance.
     *
     * @param location   The target case
     * @param drive      RoadRunner trajectory follower
     * @param leftCase   Path to follow for the left case
     * @param middleCase Path to follow for the middle case
     * @param rightCase  Path to follow for the right case
     * @param blocking   Whether trajectory following should be synchronous
     */
    public RunByCaseCommand(String location, SampleMecanumDrive drive, Object leftCase,
                            Object middleCase, Object rightCase, boolean blocking) {
        this.drive = drive;
        this.blocking = blocking;

        left = leftCase;
        middle = middleCase;
        right = rightCase;
        targetCase = location;
    }

    /**
     * Picks the path based on the provided case.
     *
     * @throws InvalidParameterException Occurs if an invalid case name is provided.
     */
    @Override
    public void initialize() throws InvalidParameterException {
        switch (targetCase.toUpperCase(Locale.ROOT)) {
            case "LEFT":
                this.followPath(left);
                break;
            case "RIGHT":
                this.followPath(right);
                break;
            case "MIDDLE":
                this.followPath(middle);
                break;
            default:
                throw new InvalidParameterException("Invalid detection case: " + targetCase.toUpperCase(Locale.ROOT));
        }
    }

    @Override
    public void execute() {
        if (!blocking)
            drive.update();
    }

    @Override
    public boolean isFinished() {
        return !drive.isBusy();
    }

    @Override
    public void cancel() {
        if (drive.isBusy())
            drive.breakFollowing();
    }

    /**
     * @param path The path object for the robot to follow
     * @throws IllegalArgumentException Occurs if the provided path isn't a RoadRunner trajectory
     */
    private void followPath(Object path) throws IllegalArgumentException {
        if (path instanceof Trajectory) {
            if (blocking)
                drive.followTrajectory((Trajectory) path);
            else drive.followTrajectoryAsync((Trajectory) path);
        } else if (path instanceof TrajectorySequence) {
            if (blocking)
                drive.followTrajectorySequence((TrajectorySequence) path);
            else drive.followTrajectorySequenceAsync((TrajectorySequence) path);
        } else
            throw new IllegalArgumentException("Invalid path type: " + path.getClass().getSimpleName() + "." +
                    "Must be either Trajectory or TrajectorySequence.");
    }
}
