package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
import org.firstinspires.ftc.teamcode.util.PropLocations;

import java.util.Locale;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Red Long", group = "auto")
public class RedLong extends CommandOpMode {

    private PropLocations location;
    private final Timing.Timer timer = new Timing.Timer(10, TimeUnit.SECONDS);

    @Override
    public void initialize() {

        TensorflowSubsystem tensorflow = new TensorflowSubsystem(hardwareMap, "Webcam 1",
                "red_prop.tflite", "Red Prop");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-34.85, -64.07, Math.toRadians(90.00));

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

        TrajectorySequence leftPurple = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-46.25, -22.93), Math.toRadians(90.00))
                .lineTo(new Vector2d(-46.25, -44.00))
                .build();
        TrajectorySequence leftYellow = drive.trajectorySequenceBuilder(leftPurple.end())
                .setReversed(true)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .splineTo(new Vector2d(7.45, -59.73), Math.toRadians(0.00))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .splineTo(new Vector2d(50.00, -28.50), Math.toRadians(0.00))
                .build();


        TrajectorySequence middlePurple = drive.trajectorySequenceBuilder(startPose)
//                .lineTo(new Vector2d(-34.85, -18.95))
                .lineTo(new Vector2d(-34.85, -40.3))
                .build();
        TrajectorySequence middleYellow = drive.trajectorySequenceBuilder(new Pose2d(-48.25, -46.25, Math.toRadians(90.00)))
                .setReversed(true)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .splineTo(new Vector2d(7.45, -59.73), Math.toRadians(0.00))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .splineTo(new Vector2d(50.00, -32.50), Math.toRadians(0.00))
                .build();


        TrajectorySequence rightPurple = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-36, -37), Math.toRadians(90))
                .turn(Math.toRadians(-90))
                .lineTo(new Vector2d(-25, -37))
                .lineTo(new Vector2d(-37, -37))
                .build();
        TrajectorySequence rightYellow = drive.trajectorySequenceBuilder(rightPurple.end())
                .setReversed(true)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .splineTo(new Vector2d(-35.76, -60.49), Math.toRadians(0.00))
                .splineTo(new Vector2d(26.14, -60.68), Math.toRadians(0.00))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .splineTo(new Vector2d(50.00, -41.5), Math.toRadians(0.00))
                .build();


        drive.setPoseEstimate(startPose);
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

                if (x < 450)
                    location = PropLocations.MIDDLE;
                else location = PropLocations.RIGHT;
            }

            telemetry.addData("FPS", tensorflow.portal.getFps());
            telemetry.addData("Current Location", location.toString());
            telemetry.addData("Confidence", String.format(Locale.US, "%.2f%%", bestDetection != null ? bestDetection.getConfidence() * 100 : 0));
            telemetry.update();
        }
        schedule(new SequentialCommandGroup(
                new InstantCommand(() -> {
                    timer.start();
                    tensorflow.shutdown();
                }),
                new InstantCommand(() -> {
                    if (location == PropLocations.MIDDLE)
                        collectorSystem.setLiftLocation(CollectorSubsystem.LiftState.STACK);
                }),
                new RunByCaseCommand(location.toString(), drive, leftPurple, middlePurple, rightPurple, false),
                new InstantCommand(() -> {
                    if (location != PropLocations.MIDDLE)
                        collectorSystem.toggleLiftLocation();
                }),
                new WaitCommand(300),
                new InstantCommand(collectorSystem::toggleLiftLocation),
                new WaitCommand(600),


                new InstantCommand(() -> collectorSystem.setLiftLocation(CollectorSubsystem.LiftState.STACK)),
                new ParallelCommandGroup( // Parallel running if no waiting is needed
                        new WaitCommand(300),
                        new WaitUntilCommand(timer::done)
                ),
                new InstantCommand(() -> {
                    if (location == PropLocations.MIDDLE)
                        drive.lineToPose(middleYellow.start());
                }),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitUntilCommand(() -> drive.getPoseEstimate().getX() > 0),
                                new InstantCommand(() -> {
                                    depositSystem.toggleSpike();
                                    depositSystem.toggleBlockers();
                                })
                        ),
                        new RunByCaseCommand(location.toString(), drive, leftYellow, middleYellow, rightYellow, false)
                ),
                new InstantCommand(() -> drive.turn(Math.toRadians(180) - drive.getPoseEstimate().getHeading(), AngleUnit.RADIANS)),
                new WaitCommand(300),


                new InstantCommand(() -> {
                    depositSystem.toggleBlockers();
                    depositSystem.toggleBlockers();
                }),
                new WaitCommand(1000),
                new InstantCommand(() -> drive.adjustPose(new Pose2d(-5, 0, 0))),
                new InstantCommand(depositSystem::toggleSpike),
                new WaitCommand(1000),

                new InstantCommand(this::terminateOpModeNow),
                new InstantCommand(() -> drive.lineToPose(new Pose2d(47, -62.2, Math.toRadians(180)))),
                new InstantCommand(() -> collectorSystem.setLiftLocation(CollectorSubsystem.LiftState.RAISED))
        ));
    }
}
