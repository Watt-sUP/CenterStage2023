package org.firstinspires.ftc.teamcode.autonomous;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.commands.RunByCaseCommand;
import org.firstinspires.ftc.teamcode.commands.subsystems.CollectorSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.TensorflowSubsystem;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Disabled
@Deprecated // Currently being replaced by BlueTest.java
@Autonomous(name = "Blue Short (Old)", group = "auto")
public class BlueShort extends CommandOpMode {

    private PropLocations location;
    private SampleMecanumDrive drive;

    @SuppressLint("DefaultLocale")
    @Override
    public void initialize() {

        TensorflowSubsystem tensorflow = new TensorflowSubsystem(hardwareMap, "Webcam 1",
                "blue_prop.tflite", "Blue Prop");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);

        telemetry.addLine("Loading trajectories...");
        telemetry.update();

        OdometrySubsystem odometrySystem = new OdometrySubsystem(
                new SimpleServo(hardwareMap, "odo_left", 0, 300),
                new SimpleServo(hardwareMap, "odo_right", 0, 300),
                new SimpleServo(hardwareMap, "odo_back", 0, 1800)
        );
        CollectorSubsystem collectorSystem = new CollectorSubsystem(
                new SimpleServo(hardwareMap, "v4b_left", 0, 180),
                new SimpleServo(hardwareMap, "v4b_right", 0, 180),
                new SimpleServo(hardwareMap, "claw", 0, 300)
        );
        DepositSubsystem depositSystem = new DepositSubsystem(
                new SimpleServo(hardwareMap, "depo_left", 0, 180),
                new SimpleServo(hardwareMap, "depo_right", 0, 180),
                new SimpleServo(hardwareMap, "stopper_top", 0, 300),
                new SimpleServo(hardwareMap, "stopper_bottom", 0, 300),
                hardwareMap.dcMotor.get("gli_sus")
        );

        TrajectorySequence leftPurple = drive.trajectorySequenceBuilder(new Pose2d(10.85, 64.07, Math.toRadians(-90.00)))
                .splineTo(new Vector2d(21.12, 22.93), Math.toRadians(-90.00))
                .lineTo(new Vector2d(21.58, 44.00))
                .build();
        Trajectory leftYellow = drive.trajectoryBuilder(leftPurple.end(), true)
                .splineTo(new Vector2d(51.5, 44.5), Math.toRadians(0.00),
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence middlePurple = drive.trajectorySequenceBuilder(new Pose2d(10.85, 64.07, Math.toRadians(-90.00)))
                .lineTo(new Vector2d(10.85, 18.95))
                .lineTo(new Vector2d(10.85, 41.3))
                .build();
        Trajectory middleYellow = drive.trajectoryBuilder(middlePurple.end(), true)
                .splineTo(new Vector2d(51.5, 37.85), Math.toRadians(0.00),
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence rightPurple = drive.trajectorySequenceBuilder(new Pose2d(10.85, 64.07, Math.toRadians(-90.00)))
                .lineTo(new Vector2d(12, 37))
                .turn(Math.toRadians(-90))
                .lineTo(new Vector2d(1, 37))
                .lineTo(new Vector2d(13.5, 37))
                .build();
        Trajectory rightYellow = drive.trajectoryBuilder(rightPurple.end(), true)
                .splineTo(new Vector2d(51.5, 30.4), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence headForStack = drive.trajectorySequenceBuilder(new Pose2d(-29.9, 62.2, Math.toRadians(180.0)), 35)
                .splineTo(new Vector2d(-59, 45.95), Math.toRadians(180.00))
                .build();

        TrajectorySequence goToBackdrop = drive.trajectorySequenceBuilder(headForStack.end())
                .setReversed(true)
                .resetVelConstraint()
                .splineTo(new Vector2d(-29.91, 62.2), Math.toRadians(0.00))
                .splineTo(new Vector2d(23.69, 62.2), Math.toRadians(0.00))
                .waitSeconds(1)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .splineTo(new Vector2d(51.5, 45), Math.toRadians(0.00))
                .build();


        drive.setPoseEstimate(rightPurple.start());
        tensorflow.setMinConfidence(0.75);
        odometrySystem.lower();

        telemetry.addLine("Ready!");
        telemetry.update();

        while (!isStarted()) {
            if (isStopRequested())
                return;

            Recognition bestDetection = tensorflow.getBestDetection();
            location = PropLocations.LEFT;

            if (bestDetection != null) {
                double x = (bestDetection.getLeft() + bestDetection.getRight()) / 2;
                location = (x < 450 ? PropLocations.MIDDLE : PropLocations.RIGHT);
            }

            telemetry.addData("FPS", tensorflow.portal.getFps());
            telemetry.addData("Current Location", location.toString());
            telemetry.addData("Confidence", String.format("%.2f%%", bestDetection != null ? bestDetection.getConfidence() * 100 : 0));
            telemetry.update();
        }

        schedule(new SequentialCommandGroup(
                // Place the purple pixel in the detected case
                new InstantCommand(tensorflow::shutdown),
                new RunByCaseCommand(location.toString(), drive, leftPurple, middlePurple, rightPurple, false),
                new InstantCommand(collectorSystem::toggleLiftLocation),
                new WaitCommand(300),
                new InstantCommand(collectorSystem::toggleLiftLocation),
                new WaitCommand(600),

                // Head for the backdrop
                new InstantCommand(() -> collectorSystem.setLiftLocation(CollectorSubsystem.LiftState.STACK)),
                new WaitCommand(300),
                new ParallelCommandGroup(
                        new InstantCommand(() -> {
                            depositSystem.toggleSpike();
                            depositSystem.toggleBlockers();
                        }),
                        new RunByCaseCommand(location.toString(), drive, leftYellow, middleYellow, rightYellow, false)
                ),
                // Correct the heading for proper pixel placement
                new InstantCommand(() -> drive.turn(Math.toRadians(180) - drive.getPoseEstimate().getHeading(), AngleUnit.RADIANS)),
                new WaitCommand(300),

                // Place the yellow pixel
                new InstantCommand(() -> {
                    depositSystem.toggleBlockers();
                    depositSystem.toggleBlockers();
                }),
                new WaitCommand(1000),
                new InstantCommand(() -> drive.adjustPose(new Pose2d(-5, 0, 0))),
                new InstantCommand(depositSystem::toggleSpike),
                new WaitCommand(1000),

                // Prepare for the cycle
                new InstantCommand(() -> {
                    drive.lineToPoseAsync(new Pose2d(47, 62.2, Math.toRadians(180)));
                    collectorSystem.setLiftLocation(CollectorSubsystem.LiftState.RAISED);
                }),
                new WaitUntilCommand(() -> !drive.isBusy()),
                new InstantCommand(() -> drive.turn(Math.toRadians(180) - drive.getPoseEstimate().getHeading(), AngleUnit.RADIANS)),

                // Head for the other side of the field
                new InstantCommand(() -> drive.lineToPoseAsync(new Pose2d(-29.9, 62.2, Math.toRadians(180.0)))),
                new WaitUntilCommand(() -> !drive.isBusy()),
                new ConditionalCommand(
                        // If you made it through, head for the stacks and come back to the backdrop
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> collectorSystem.setLiftLocation(CollectorSubsystem.LiftState.STACK)).andThen(
                                                new WaitCommand(300),
                                                new InstantCommand(collectorSystem::toggleClamp),
                                                new WaitUntilCommand(() -> !drive.isBusy())
                                        ),

                                        new InstantCommand(collectorSystem::toggleClamp).andThen(new WaitCommand(300)),
                                        new InstantCommand(() -> drive.followTrajectorySequenceAsync(goToBackdrop)).andThen(new WaitUntilCommand(() -> drive.getPoseEstimate().getX() > 0)),
                                        new InstantCommand(() -> collectorSystem.setLiftLocation(CollectorSubsystem.LiftState.RAISED)).andThen(
                                                new InstantCommand(collectorSystem::toggleClamp),
                                                new WaitCommand(300),
                                                new InstantCommand(depositSystem::toggleBlockers)
                                        ),

                                        new InstantCommand(() -> collectorSystem.setLiftLocation(CollectorSubsystem.LiftState.STACK)).andThen(
                                                new WaitCommand(250),
                                                new InstantCommand(depositSystem::toggleSpike),
                                                new WaitCommand(500),
                                                new WaitUntilCommand(() -> !drive.isBusy())
                                        ),
                                        new InstantCommand(() -> drive.turn(Math.toRadians(180) - drive.getPoseEstimate().getHeading(), AngleUnit.RADIANS)).andThen(
                                                new InstantCommand(depositSystem::toggleBlockers),
                                                new WaitCommand(400)
                                        ),
                                        new InstantCommand(() -> drive.adjustPoseAsync(new Pose2d(-5, 0, 0))).andThen(new WaitUntilCommand(() -> !drive.isBusy())),
                                        new InstantCommand(() -> drive.adjustPoseAsync(new Pose2d(5, 0, 0))).andThen(
                                                new WaitUntilCommand(() -> !drive.isBusy()),
                                                new InstantCommand(depositSystem::toggleBlockers),
                                                new WaitCommand(700)
                                        ),

                                        new InstantCommand(() -> drive.adjustPoseAsync(new Pose2d(-5, 0, 0))).andThen(
                                                new WaitUntilCommand(() -> !drive.isBusy()),
                                                new InstantCommand(depositSystem::toggleSpike),
                                                new WaitCommand(1000),
                                                new InstantCommand(() -> collectorSystem.setLiftLocation(CollectorSubsystem.LiftState.RAISED))
                                        )
//                                        new InstantCommand(() -> drive.lineToPoseAsync(new Pose2d(47, 62.2, Math.toRadians(180))))
                                ),
                                new InstantCommand(() -> drive.followTrajectorySequenceAsync(headForStack))
                        ),
                        // Go back to parking if you got stuck
                        new InstantCommand(() -> drive.lineToPoseAsync(new Pose2d(47, 62.2, Math.toRadians(180)))),
                        () -> drive.getPoseEstimate().getX() < -24 // Check if you're past the stage
                )
        ));
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        drive.update();
    }

    private enum PropLocations {
        LEFT,
        MIDDLE,
        RIGHT
    }
}
