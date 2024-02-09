package org.firstinspires.ftc.teamcode.autonomous.red;

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

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.autonomous.PathGenerator;
import org.firstinspires.ftc.teamcode.autonomous.assets.AllianceColor;
import org.firstinspires.ftc.teamcode.autonomous.assets.BackstageRoute;
import org.firstinspires.ftc.teamcode.autonomous.assets.PropLocations;
import org.firstinspires.ftc.teamcode.autonomous.assets.Stack;
import org.firstinspires.ftc.teamcode.autonomous.assets.StartingPosition;
import org.firstinspires.ftc.teamcode.commands.RunByCaseCommand;
import org.firstinspires.ftc.teamcode.commands.subsystems.CollectorSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.TensorflowSubsystem;
import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Locale;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Red Long", group = "auto")
public class RedLong extends CommandOpMode {

    private PropLocations location;
    private final Timing.Timer teammate = new Timing.Timer(7, TimeUnit.SECONDS);

    @Override
    public void initialize() {

        TensorflowSubsystem tensorflow = new TensorflowSubsystem(hardwareMap, "Webcam 1",
                "red_prop.tflite", "Red Prop");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        PathGenerator generator = new PathGenerator(drive);

        generator.setStartingLocation(AllianceColor.RED, StartingPosition.AUDIENCE);
        tensorflow.setMinConfidence(0.8);

        telemetry.addLine("Loading trajectories...");
        telemetry.update();

        OdometrySubsystem odometrySystem = new OdometrySubsystem(
                new SimpleServo(hardwareMap, "odo_left", 0, 180),
                new SimpleServo(hardwareMap, "odo_right", 0, 180),
                new SimpleServo(hardwareMap, "odo_back", 0, 1800)
        );
        CollectorSubsystem collectorSystem = new CollectorSubsystem(
                new SimpleServo(hardwareMap, "v4b_left", 0, 180),
                new SimpleServo(hardwareMap, "v4b_right", 0, 180),
                new SimpleServo(hardwareMap, "claw", 0, 300)
        );
        DepositSubsystem depositSystem = new DepositSubsystem(
                new SimpleServo(hardwareMap, "depo_left", 0, 220),
                new SimpleServo(hardwareMap, "depo_right", 0, 220),
                new SimpleServo(hardwareMap, "stopper_top", 0, 300),
                new SimpleServo(hardwareMap, "stopper_bottom", 0, 300),
                hardwareMap.dcMotor.get("gli_sus")
        );

        TrajectorySequence leftPurple = drive.trajectorySequenceBuilder(generator.getStartingPose())
                .splineTo(new Vector2d(-47.5, -32)
                        .plus(new Vector2d(0, -13).rotated(Math.toRadians(30))), Math.toRadians(120.00))
                .build();
        TrajectorySequence leftYellow = drive.trajectorySequenceBuilder(leftPurple.end(), 40)
                .setReversed(true)
                .splineTo(new Vector2d(-24.00, -60.00), Math.toRadians(0.00))
                .splineTo(new Vector2d(2.12, -60.00), Math.toRadians(0.00))
                .splineTo(new Vector2d(50.00, -28.50), Math.toRadians(0.00))
                .build();


        TrajectorySequence middlePurple = drive.trajectorySequenceBuilder(generator.getStartingPose())
                .splineTo(new Vector2d(-37.00, -38.00), Math.toRadians(90.00))
                .build();
        TrajectorySequence middleYellow = drive.trajectorySequenceBuilder(middlePurple.end(), 40)
                .setReversed(true)
                .splineTo(new Vector2d(-24.00, -60.00), Math.toRadians(0.00))
                .splineTo(new Vector2d(2.12, -60.00), Math.toRadians(0.00))
                .splineTo(new Vector2d(50.00, -35.25), Math.toRadians(0.00))
                .build();


        TrajectorySequence rightPurple = drive.trajectorySequenceBuilder(generator.getStartingPose())
                .splineTo(new Vector2d(-24.5, -35)
                        .plus(new Vector2d(0, -12).rotated(Math.toRadians(-45))), Math.toRadians(45.00))
                .build();
        TrajectorySequence rightYellow = drive.trajectorySequenceBuilder(rightPurple.end(), 40)
                .setReversed(true)
                .splineTo(new Vector2d(-24.00, -60.00), Math.toRadians(0.00))
                .splineTo(new Vector2d(2.12, -60.00), Math.toRadians(0.00))
                .splineTo(new Vector2d(50.00, -39.50), Math.toRadians(0.00))
                .build();

        TrajectorySequence stackLeft = generator.generateStackPath(leftYellow.end(), Stack.FAR);
        TrajectorySequence stackMid = generator.generateStackPath(middleYellow.end(), Stack.CLOSE);
        TrajectorySequence stackRight = generator.generateStackPath(rightYellow.end(), Stack.CLOSE);

        TrajectorySequence backdropSide = generator.generateBackstagePath(stackMid.end(), BackstageRoute.SIDE);
        TrajectorySequence backdropCenter = generator.generateBackstagePath(stackLeft.end(), BackstageRoute.CENTER);

        odometrySystem.lower();
        telemetry.addLine("Ready!");
        telemetry.update();

        while (!isStarted()) {
            if (isStopRequested())
                return;

            Recognition bestDetection = tensorflow.getBestDetection();
            location = PropLocations.RIGHT;

            if (bestDetection != null) {
                double x = (bestDetection.getLeft() + bestDetection.getRight()) / 2.0;
                location = x < (bestDetection.getImageWidth() / 2.0) ? PropLocations.LEFT : PropLocations.MIDDLE;
            }

            telemetry.addData("FPS", tensorflow.portal.getFps());
            telemetry.addData("Current Location", location.toString());
            telemetry.addData("Confidence", String.format(Locale.US, "%.2f%%", bestDetection != null ? bestDetection.getConfidence() * 100 : 0));
            telemetry.update();
        }

        tensorflow.shutdown();
        schedule(new SequentialCommandGroup(
                new InstantCommand(() -> {
                    collectorSystem.setLiftLocation(CollectorSubsystem.LiftState.STACK);
                    teammate.start();
                }),
                new RunByCaseCommand(location.toString(), drive, leftPurple, middlePurple, rightPurple, true),
                new InstantCommand(collectorSystem::toggleLiftLocation).andThen(
                        new WaitCommand(300),
                        new InstantCommand(() -> {
                            collectorSystem.setClampPosition(90);
                            collectorSystem.setLiftLocation(CollectorSubsystem.LiftState.STACK);
                        })
                ),
                new WaitUntilCommand(teammate::done),
                new ParallelCommandGroup(
                        new WaitUntilCommand(() -> drive.getPoseEstimate().getX() > 0)
                                .andThen(
                                        new InstantCommand(() -> {
                                            depositSystem.toggleSpike();
                                            depositSystem.toggleBlockers();
                                        }),
                                        new WaitCommand(300),
                                        new InstantCommand(() -> depositSystem.setSlidesTicks(50))
                                ),
                        new RunByCaseCommand(location.toString(), drive, leftYellow, middleYellow, rightYellow, false)
                ),
                new WaitCommand(300),
                new InstantCommand(depositSystem::toggleBlockers).andThen(
                        new WaitCommand(300),
                        new InstantCommand(depositSystem::toggleBlockers)
                ),
                new WaitCommand(1000),
                new InstantCommand(depositSystem::toggleSpike),
                new WaitCommand(500),
//                new RunByCaseCommand(location.toString(), drive, stackLeft, stackMid, stackRight, true)
//                        .andThen(
//                                new InstantCommand(collectorSystem::toggleClamp),
//                                new WaitCommand(1250),
//                                new InstantCommand(collectorSystem::toggleClamp)
//                        ),
//                new InstantCommand(() -> drive.followTrajectorySequenceAsync(location == PropLocations.LEFT ? backdropCenter : backdropSide)),
//                new ParallelCommandGroup(
//                        new RunCommand(drive::update).interruptOn(() -> !drive.isBusy()),
//                        new WaitUntilCommand(() -> drive.getPoseEstimate().getX() > 0)
//                                .andThen(
//                                        new InstantCommand(() -> collectorSystem.setLiftLocation(CollectorSubsystem.LiftState.STACK)),
//                                        new WaitCommand(300),
//                                        new InstantCommand(() -> {
//                                            depositSystem.toggleBlockers();
//                                            depositSystem.toggleSpike();
//                                        }),
//                                        new WaitCommand(300),
//                                        new InstantCommand(() -> depositSystem.setSlidesTicks(200))
//                                )
//                ),
//                new InstantCommand(depositSystem::toggleBlockers)
//                        .andThen(
//                                new WaitCommand(600),
//                                new InstantCommand(depositSystem::toggleSpike),
//                                new WaitCommand(300),
//                                new InstantCommand(depositSystem::toggleSpike),
//                                new WaitCommand(700)
//                        ),
//                new InstantCommand(depositSystem::toggleBlockers)
//                        .andThen(
//                                new WaitCommand(1000),
//                                new InstantCommand(depositSystem::toggleSpike)
//                        ),
                new WaitCommand(1000)
                        .andThen(
                                new InstantCommand(() -> depositSystem.setSlidesPosition(0)),
                                new WaitUntilCommand(() -> depositSystem.getSlidesTicks() < 25),
                                new InstantCommand(() -> collectorSystem.setLiftLocation(CollectorSubsystem.LiftState.RAISED))
                        ),

                new InstantCommand(() -> drive.adjustPose(new Pose2d(-5, 0, 0))),
                new InstantCommand(() -> drive.lineToPose(new Pose2d(48, -12, Math.toRadians(180)))),
                new InstantCommand(() -> drive.adjustPose(new Pose2d(10, 0, 0)))
        ));
    }
}
