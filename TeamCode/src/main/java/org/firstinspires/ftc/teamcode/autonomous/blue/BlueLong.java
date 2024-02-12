package org.firstinspires.ftc.teamcode.autonomous.blue;

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
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Mugurel;
import org.firstinspires.ftc.teamcode.autonomous.PathGenerator;
import org.firstinspires.ftc.teamcode.autonomous.assets.AllianceColor;
import org.firstinspires.ftc.teamcode.autonomous.assets.BackstageRoute;
import org.firstinspires.ftc.teamcode.autonomous.assets.PropLocations;
import org.firstinspires.ftc.teamcode.autonomous.assets.Stack;
import org.firstinspires.ftc.teamcode.autonomous.assets.StartingPosition;
import org.firstinspires.ftc.teamcode.commands.RunByCaseCommand;
import org.firstinspires.ftc.teamcode.commands.subsystems.CollectorSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.TensorflowSubsystem;
import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Locale;
import java.util.Map;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Blue Long", group = "auto")
public class BlueLong extends CommandOpMode {

    private PropLocations location;
    private final Timing.Timer teammate = new Timing.Timer(12, TimeUnit.SECONDS);

    @Override
    public void initialize() {

        TensorflowSubsystem tensorflow = new TensorflowSubsystem(hardwareMap, "Webcam 1",
                "blue_prop.tflite", "Blue Prop");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        PathGenerator generator = new PathGenerator(drive);

        generator.setStartingLocation(AllianceColor.BLUE, StartingPosition.AUDIENCE);
        tensorflow.setMinConfidence(0.8);

        telemetry.addLine("Loading trajectories...");
        telemetry.update();

        Mugurel robot = new Mugurel(hardwareMap, Mugurel.OpModeType.AUTO);
        CollectorSubsystem collectorSystem = robot.getSubsystem(CollectorSubsystem.class);
        DepositSubsystem depositSystem = robot.getSubsystem(DepositSubsystem.class);

        Map<PropLocations, TrajectorySequence> purpleCases = generator.generatePurpleCases();


        TrajectorySequence rightYellow = drive.trajectorySequenceBuilder(purpleCases.get(PropLocations.RIGHT).end(), 40)
                .setReversed(true)
                .splineTo(new Vector2d(-24.00, 60.00), Math.toRadians(0.00))
                .splineTo(new Vector2d(2.12, 60.00), Math.toRadians(0.00))
                .splineTo(new Vector2d(50.00, 29.50), Math.toRadians(0.00))
                .build();

        TrajectorySequence middleYellow = drive.trajectorySequenceBuilder(purpleCases.get(PropLocations.MIDDLE).end(), 40)
                .setReversed(true)
                .splineTo(new Vector2d(-24.00, 60.00), Math.toRadians(0.00))
                .splineTo(new Vector2d(2.12, 60.00), Math.toRadians(0.00))
                .splineTo(new Vector2d(50.00, 35.50), Math.toRadians(0.00))
                .build();

        TrajectorySequence leftYellow = drive.trajectorySequenceBuilder(purpleCases.get(PropLocations.LEFT).end(), 40)
                .setReversed(true)
                .splineTo(new Vector2d(-24.00, 60.00), Math.toRadians(0.00))
                .splineTo(new Vector2d(2.12, 60.00), Math.toRadians(0.00))
                .splineTo(new Vector2d(50.00, 40.00), Math.toRadians(0.00))
                .build();

        telemetry.addLine("Ready!");
        telemetry.update();

        while (!isStarted()) {
            if (isStopRequested())
                return;

            Recognition bestDetection = tensorflow.getBestDetection();
            location = PropLocations.LEFT;

            if (bestDetection != null) {
                double x = (bestDetection.getLeft() + bestDetection.getRight()) / 2.0;
                location = x < (bestDetection.getImageWidth() / 2.0) ? PropLocations.MIDDLE : PropLocations.RIGHT;
            }

            telemetry.addData("FPS", tensorflow.portal.getFps());
            telemetry.addData("Current Location", location.toString());
            telemetry.addData("Confidence", String.format(Locale.US, "%.2f%%", bestDetection != null ? bestDetection.getConfidence() * 100 : 0));
            telemetry.update();
        }

        TrajectorySequence stackLeft = generator.generateStackPath(leftYellow.end(), Stack.CLOSE);
        TrajectorySequence stackMid = generator.generateStackPath(middleYellow.end(), Stack.CLOSE);
        TrajectorySequence stackRight = generator.generateStackPath(rightYellow.end(), Stack.CLOSE);

        TrajectorySequence backdropSide = generator.generateBackstagePath(stackMid.end(), BackstageRoute.SIDE);
        TrajectorySequence backdropRight = generator.generateBackstagePath(stackRight.end().plus(new Pose2d(0, 5, 0)), BackstageRoute.SIDE);

        tensorflow.shutdown();
        schedule(new SequentialCommandGroup(
                new InstantCommand(() -> {
                    teammate.start();
                    collectorSystem.setLiftLocation(CollectorSubsystem.LiftState.STACK);
                }),
                new RunByCaseCommand(location, purpleCases, drive, true),
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
                                        })
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
//                new InstantCommand(() -> {
//                    if (location == PropLocations.RIGHT)
//                        drive.adjustPose(new Pose2d(0, 5, 0));
//                }),
//                new InstantCommand(() -> drive.followTrajectorySequenceAsync(location != PropLocations.RIGHT ? backdropSide : backdropRight)),
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
//                new InstantCommand(() -> {
//                    if (location == PropLocations.RIGHT)
//                        drive.adjustPose(new Pose2d(0, 5, 0));
//                }),
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
//
//                new WaitCommand(1000)
//                        .andThen(new InstantCommand(() -> depositSystem.setSlidesPosition(0))),

                new InstantCommand(() -> drive.adjustPose(new Pose2d(-5, 0, 0))),
                new InstantCommand(() -> drive.lineToPose(new Pose2d(48, 12, Math.toRadians(180))))
                        .andThen(new InstantCommand(() -> collectorSystem.setLiftLocation(CollectorSubsystem.LiftState.RAISED))),
                new InstantCommand(() -> drive.adjustPose(new Pose2d(10, 0, 0)))
        ));
    }
}
