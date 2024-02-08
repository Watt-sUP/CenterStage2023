package org.firstinspires.ftc.teamcode.autonomous.red;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.hardware.SimpleServo;
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
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Locale;

@Autonomous(name = "Red Short (Side)")
public class RedShortSide extends CommandOpMode {
    private PropLocations location = PropLocations.LEFT;

    @Override
    public void initialize() {
        TensorflowSubsystem tensorflow = new TensorflowSubsystem(hardwareMap, "Webcam 1",
                "red_prop.tflite", "Red Prop");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Loading trajectories...");
        telemetry.update();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        PathGenerator generator = new PathGenerator(drive);

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

        generator.setStartingLocation(AllianceColor.RED, StartingPosition.BACKDROP);
        tensorflow.setMinConfidence(0.8);
        odometrySystem.lower();

        Trajectory middlePurple = drive.trajectoryBuilder(generator.getStartingPose())
                .splineTo(new Vector2d(15, -38), Math.toRadians(90.00))
                .build();
        Trajectory leftPurple = drive.trajectoryBuilder(generator.getStartingPose())
                .splineTo(new Vector2d(.5, -33)
                        .minus(Vector2d.polar(12, Math.toRadians(135))), Math.toRadians(135.00))
                .build();
        Trajectory rightPurple = drive.trajectoryBuilder(generator.getStartingPose())
                .splineTo(new Vector2d(23.5, -32)
                        .minus(Vector2d.polar(13, Math.toRadians(60))), Math.toRadians(60.00))
                .build();

        Trajectory rightYellow = drive.trajectoryBuilder(rightPurple.end(), true)
                .splineTo(new Vector2d(31.05, -53.32), Math.toRadians(0.00))
                .splineTo(new Vector2d(50.00, -43.00), Math.toRadians(0.00))
                .build();
        Trajectory leftYellow = drive.trajectoryBuilder(leftPurple.end(), true)
                .splineTo(new Vector2d(50.00, -29.50), Math.toRadians(0.00))
                .build();
        Trajectory middleYellow = drive.trajectoryBuilder(middlePurple.end(), true)
                .splineTo(new Vector2d(50.00, -35.5), Math.toRadians(0.00))
                .build();

        TrajectorySequence stackLeft = generator.generateStackPath(leftYellow.end(), Stack.FAR);
        TrajectorySequence stackMid = generator.generateStackPath(middleYellow.end(), Stack.CLOSE);
        TrajectorySequence stackRight = generator.generateStackPath(rightYellow.end(), Stack.CLOSE);

        TrajectorySequence backdropSide = generator.generateBackstagePath(stackMid.end(), BackstageRoute.SIDE);
        TrajectorySequence backdropCenter = generator.generateBackstagePath(stackLeft.end(), BackstageRoute.CENTER);

        while (!isStarted()) {
            if (isStopRequested())
                return;

            Recognition bestDetection = tensorflow.getBestDetection();
            location = PropLocations.LEFT;

            if (bestDetection != null) {
                double x = (bestDetection.getLeft() + bestDetection.getRight()) / 2;
                location = x < 450 ? PropLocations.MIDDLE : PropLocations.RIGHT;
            }

            telemetry.addData("FPS", tensorflow.portal.getFps());
            telemetry.addData("Current Location", location.toString());
            telemetry.addData("Confidence", String.format(Locale.US, "%.2f%%", bestDetection != null ? bestDetection.getConfidence() * 100 : 0));
            telemetry.update();
        }

        generator.setPropLocation(location);
        tensorflow.shutdown();

        schedule(new SequentialCommandGroup(
                new InstantCommand(() -> collectorSystem.setLiftLocation(CollectorSubsystem.LiftState.STACK)),
                new RunByCaseCommand(location.toString(), drive, leftPurple, middlePurple, rightPurple, true),
                new InstantCommand(collectorSystem::toggleLiftLocation).andThen(
                        new WaitCommand(300),
                        new InstantCommand(() -> {
                            collectorSystem.setClampPosition(90);
                            collectorSystem.setLiftLocation(CollectorSubsystem.LiftState.STACK);
                            depositSystem.toggleBlockers();
                            depositSystem.toggleSpike();
                        })
                ),
                new RunByCaseCommand(location.toString(), drive, leftYellow, middleYellow, rightYellow, true),
                new InstantCommand(depositSystem::toggleBlockers).andThen(
                        new WaitCommand(300),
                        new InstantCommand(depositSystem::toggleBlockers),
                        new WaitCommand(700),
                        new InstantCommand(depositSystem::toggleSpike)
                ),
//                new RunByCaseCommand(location.toString(), drive, stackLeft, stackMid, stackRight, true)
//                        .andThen(
//                                new InstantCommand(collectorSystem::toggleClamp),
//                                new WaitCommand(1250),
//                                new InstantCommand(collectorSystem::toggleClamp)
//                        ),
//                new InstantCommand(() -> drive.followTrajectorySequenceAsync(location != PropLocations.LEFT ? backdropSide : backdropCenter)),
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
//                new WaitCommand(1000)
//                        .andThen(
//                                new InstantCommand(() -> depositSystem.setSlidesPosition(0)),
//                        ),

                new InstantCommand(() -> drive.adjustPose(new Pose2d(-5, 0, 0))),
                new InstantCommand(() -> drive.lineToPose(new Pose2d(48, -60, Math.toRadians(180))))
                        .andThen(new InstantCommand(() -> collectorSystem.setLiftLocation(CollectorSubsystem.LiftState.RAISED))),
                new InstantCommand(() -> drive.adjustPose(new Pose2d(10, 0, 0)))
        ));
    }
}
