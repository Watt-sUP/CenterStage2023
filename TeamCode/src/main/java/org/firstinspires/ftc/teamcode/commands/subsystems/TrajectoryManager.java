package org.firstinspires.ftc.teamcode.commands.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Function;

public class TrajectoryManager extends SubsystemBase {

    private final SampleMecanumDrive drive;
    private final Map<String, TrajectorySequence> trajectories;

    public TrajectoryManager(HardwareMap hardwareMap) {
        drive = new SampleMecanumDrive(hardwareMap);
        trajectories = new LinkedHashMap<>();
    }

    public void setPoseEstimate(Pose2d poseEstimate) {
        drive.setPoseEstimate(poseEstimate);
    }

    public void updateDrive() {
        drive.update();
    }

    public void runDriveAction(Consumer<SampleMecanumDrive> action) {
        action.accept(drive);
    }

    /**
     * Checks whether the robot is currently idle.
     *
     * @return True if the robot isn't following a trajectory
     */
    public boolean isFree() {
        return !drive.isBusy();
    }

    /**
     * Stops the trajectory the robot is currently following.
     */
    public void stopCurrentTrajectory() {
        RobotLog.d("TrajectoryManager: Ended trajectory following early.");
        drive.breakFollowing();
    }

    /**
     * <p>Adds a trajectory for future execution.</p>
     * <p>Both Trajectory and TrajectorySequence types are supported.</p>
     *
     * @param name               The ID of the trajectory
     * @param trajectorySupplier A function that returns the trajectory to be added
     * @throws IllegalArgumentException If the return type is neither Trajectory or TrajectorySequence
     */
    public void addTrajectory(String name, Function<SampleMecanumDrive, Object> trajectorySupplier) throws IllegalArgumentException {
        Object trajectoryObject = trajectorySupplier.apply(drive);

        if (trajectoryObject instanceof TrajectorySequence)
            trajectories.put(name, (TrajectorySequence) trajectoryObject);
        else if (trajectoryObject instanceof Trajectory) {
            Trajectory trajectory = (Trajectory) trajectoryObject;
            trajectories.put(name,
                    drive.trajectorySequenceBuilder(trajectory.start())
                            .addTrajectory(trajectory)
                            .build()
            );
        } else throw new IllegalArgumentException("Cannot convert '" + name +
                "' of type " + trajectoryObject.getClass().getSimpleName() + " to TrajectorySequence");

        RobotLog.d("TrajectoryManager: " + trajectoryObject.getClass().getSimpleName() +
                " object with ID '" + name + " added successfully");
    }

    /**
     * Runs a trajectory in a blocking manner or the begins asynchronous following of one.
     *
     * @param name     The ID of the trajectory to follow
     * @param blocking Whether the trajectory should be followed in a synchronous manner
     * @throws IllegalArgumentException If a trajectory matching the ID isn't found
     */
    public void runTrajectory(String name, boolean blocking) throws IllegalArgumentException {
        if (!trajectories.containsKey(name))
            throw new IllegalArgumentException("Trajectory '" + name + "' does not exist");

        if (drive.isBusy())
            this.stopCurrentTrajectory();

        RobotLog.d("TrajectoryManager: Running trajectory " + name + "...");
        if (blocking)
            drive.followTrajectorySequence(trajectories.get(name));
        else drive.followTrajectorySequenceAsync(trajectories.get(name));

        trajectories.remove(name);
    }

    /**
     * Deletes a multitude of trajectories. Names that aren't found are ignored.
     *
     * @param names The ID of the trajectories to remove
     */
    public void bulkDelete(String... names) {
        for (String name : names)
            trajectories.remove(name);
    }

    /**
     * Retrieves the end pose of a stored trajectory.
     *
     * @param name The ID of the target trajectory
     * @return The end pose of the trajectory
     * @throws IllegalArgumentException If the trajectory is not found
     */
    public Pose2d getTrajectoryEnd(String name) {
        if (!trajectories.containsKey(name))
            throw new IllegalArgumentException("Trajectory '" + name + "' does not exist");

        return trajectories.get(name).end();
    }
}
