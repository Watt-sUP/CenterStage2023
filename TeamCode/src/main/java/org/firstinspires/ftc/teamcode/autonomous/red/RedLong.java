package org.firstinspires.ftc.teamcode.autonomous.red;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
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
import org.firstinspires.ftc.teamcode.roadrunner.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.HashMap;
import java.util.Locale;
import java.util.Map;

@Autonomous(name = "Red Long", group = "auto")
public class RedLong extends CommandOpMode {

    private PropLocations location;

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

        Mugurel robot = new Mugurel(hardwareMap, Mugurel.OpModeType.AUTO);
        CollectorSubsystem intake = robot.getSubsystem(CollectorSubsystem.class);
        DepositSubsystem outtake = robot.getSubsystem(DepositSubsystem.class);

        TrajectorySequence leftPurple = drive.trajectorySequenceBuilder(generator.getStartingPose())
                .splineToSplineHeading(new Pose2d(
                        new Vector2d(-47.5, -33).minus(Vector2d.polar(13, Math.toRadians(180))),
                        Math.toRadians(180)
                ), Math.toRadians(90.00))
                .build();

        TrajectorySequence middlePurple = drive.trajectorySequenceBuilder(generator.getStartingPose())
                .splineTo(new Vector2d(-39.00, -38.00), Math.toRadians(90.00))
                .build();

        TrajectorySequence rightPurple = drive.trajectorySequenceBuilder(generator.getStartingPose())
                .splineTo(new Vector2d(-24.5, -33)
                        .minus(Vector2d.polar(11, Math.toRadians(30))), Math.toRadians(30.00))
                .build();

        TrajectorySequence whiteRight = drive.trajectorySequenceBuilder(rightPurple.end())
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-52.50, -35.75, Math.toRadians(180.00)), Math.toRadians(180))
                .addTemporalMarker(() ->
                {
                    intake.setLiftLocation(CollectorSubsystem.LiftState.STACK);
                    intake.adjustLiftPosition(-0.02);
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    if (intake.clamping != CollectorSubsystem.ClampState.OPENED)
                        intake.toggleClamp();
                })
                .setConstraints(
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30)
                )
                .lineToLinearHeading(new Pose2d(-56.50, -35.75, Math.toRadians(180.00)))
                .addTemporalMarker(intake::toggleClamp)
                .waitSeconds(0.5)
                .build();

        TrajectorySequence whiteMiddle = drive.trajectorySequenceBuilder(middlePurple.end())
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-52.50, -35.75, Math.toRadians(180.00)), Math.toRadians(180))
                .addTemporalMarker(() ->
                {
                    intake.setLiftLocation(CollectorSubsystem.LiftState.STACK);
                    intake.adjustLiftPosition(-0.02);
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    if (intake.clamping != CollectorSubsystem.ClampState.OPENED)
                        intake.toggleClamp();
                })
                .setConstraints(
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30)
                )
                .lineToLinearHeading(new Pose2d(-56.50, -35.75, Math.toRadians(180.00)))
                .addTemporalMarker(intake::toggleClamp)
                .waitSeconds(0.5)
                .build();

        TrajectorySequence whiteLeft = drive.trajectorySequenceBuilder(leftPurple.end())
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-48, -16, Math.toRadians(180)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-52.5, -23.5, Math.toRadians(180)), Math.toRadians(-90))
                .addTemporalMarker(() -> {
                    intake.setLiftLocation(CollectorSubsystem.LiftState.STACK);
                    intake.adjustLiftPosition(-0.02);
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    if (intake.clamping != CollectorSubsystem.ClampState.OPENED)
                        intake.toggleClamp();
                })
                .setConstraints(
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30)
                )
                .lineToLinearHeading(new Pose2d(-56.50, -23.5, Math.toRadians(180.00)))
                .addTemporalMarker(intake::toggleClamp)
                .waitSeconds(0.5)
                .build();

        Map<PropLocations, TrajectorySequence> backdrops = new HashMap<PropLocations, TrajectorySequence>() {{
            put(PropLocations.LEFT,
                    drive.trajectorySequenceBuilder(whiteLeft.end())
                            .lineToLinearHeading(new Pose2d(-36, -23.5, Math.toRadians(180)))
                            .setTangent(Math.toRadians(-90))
                            .setConstraints(
                                    SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(45)
                            )
                            .splineToSplineHeading(new Pose2d(-24, -59, Math.toRadians(180)), Math.toRadians(0))
                            .splineTo(new Vector2d(2.12, -59.00), Math.toRadians(0.00))
                            .splineTo(new Vector2d(50.50, -35.50), Math.toRadians(0.00))
                            .build()
            );
            put(PropLocations.MIDDLE, generator.generateBackstagePath(whiteMiddle.end(), new Vector2d(50.50, -42.50), BackstageRoute.SIDE));
            put(PropLocations.RIGHT, generator.generateBackstagePath(whiteRight.end(), BackstageRoute.SIDE));
        }};
        Map<PropLocations, Vector2d> yellowLocation = new HashMap<PropLocations, Vector2d>() {{
            put(PropLocations.LEFT, new Vector2d(50.50, -29.00));
            put(PropLocations.MIDDLE, new Vector2d(50.50, -37.00));
            put(PropLocations.RIGHT, new Vector2d(50.50, -44.00));
        }};
        TrajectorySequence stackLeft = generator.generateStackPath(new Pose2d(yellowLocation.get(PropLocations.LEFT), Math.toRadians(180)), Stack.CLOSE);
        TrajectorySequence stackRight = generator.generateStackPath(new Pose2d(yellowLocation.get(PropLocations.RIGHT), Math.toRadians(180.00)), Stack.CLOSE);
        TrajectorySequence stackMiddle = generator.generateStackPath(new Pose2d(yellowLocation.get(PropLocations.MIDDLE), Math.toRadians(180.00)), Stack.CLOSE);

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
                new InstantCommand(() -> intake.setLiftLocation(CollectorSubsystem.LiftState.STACK)),
                new RunByCaseCommand(location.toString(), drive, leftPurple, middlePurple, rightPurple, true),
                new InstantCommand(intake::toggleLiftLocation).andThen(
                        new WaitCommand(300),
                        new InstantCommand(() -> intake.setLiftLocation(CollectorSubsystem.LiftState.RAISED))
                ),
                new RunByCaseCommand(location.toString(), drive, whiteLeft, whiteMiddle, whiteRight, true),
                new InstantCommand(() -> drive.followTrajectorySequenceAsync(backdrops.get(location))),
                new ParallelCommandGroup(
                        new RunCommand(drive::update).interruptOn(() -> !drive.isBusy()),
                        new WaitCommand(700).andThen(new InstantCommand(intake::toggleClamp)),
                        new WaitUntilCommand(() -> drive.getPoseEstimate().getX() > 0)
                                .andThen(
                                        new InstantCommand(() -> intake.setLiftLocation(CollectorSubsystem.LiftState.STACK)),
                                        new WaitCommand(300),
                                        new InstantCommand(() -> {
                                            intake.setClampPosition(90);
                                            intake.adjustLiftPosition(0.02);
                                            outtake.toggleBlockers();
                                            outtake.toggleSpike();
                                        })
                                )
                ),
                new InstantCommand(outtake::toggleBlockers).andThen(
                        new WaitCommand(500),
                        new InstantCommand(outtake::toggleSpike),
                        new WaitCommand(300),
                        new InstantCommand(outtake::toggleSpike)
                ),
                new InstantCommand(() -> drive.lineToPose(new Pose2d(yellowLocation.get(location), Math.toRadians(180.00))))
                        .andThen(
                                new InstantCommand(outtake::toggleBlockers),
                                new WaitCommand(500),
                                new InstantCommand(outtake::toggleSpike),
                                new WaitCommand(300),
                                new InstantCommand(() -> outtake.setSlidesPosition(0))
                        ),
                new RunByCaseCommand(location.toString(), drive, stackLeft, stackMiddle, stackRight, true)
                        .andThen(
                                new InstantCommand(intake::toggleClamp),
                                new WaitCommand(500)
                        ),
                new InstantCommand(() -> drive.followTrajectorySequenceAsync(backdrops.get(location))),
                new ParallelCommandGroup(
                        new RunCommand(drive::update).interruptOn(() -> !drive.isBusy()),
                        new WaitCommand(700).andThen(new InstantCommand(intake::toggleClamp)),
                        new WaitUntilCommand(() -> drive.getPoseEstimate().getX() > 0)
                                .andThen(
                                        new InstantCommand(() -> intake.setLiftLocation(CollectorSubsystem.LiftState.STACK)),
                                        new WaitCommand(300),
                                        new InstantCommand(() -> {
                                            outtake.setSlidesTicks(200);
                                            outtake.toggleBlockers();
                                            outtake.toggleSpike();
                                        })
                                )
                ),
                new InstantCommand(outtake::toggleBlockers)
                        .andThen(
                                new WaitCommand(500),
                                new InstantCommand(outtake::toggleSpike),
                                new WaitCommand(300),
                                new InstantCommand(outtake::toggleSpike),
                                new WaitCommand(300)
                        ),
                new InstantCommand(outtake::toggleBlockers)
                        .andThen(
                                new WaitCommand(500),
                                new InstantCommand(outtake::toggleSpike)
                        ),
                new WaitCommand(500)
                        .andThen(new InstantCommand(() -> outtake.setSlidesPosition(0))),

                new InstantCommand(() -> drive.adjustPose(new Pose2d(-5, 0, 0))),
                new InstantCommand(() -> drive.lineToPose(new Pose2d(48, -60, Math.toRadians(180))))
                        .andThen(new InstantCommand(() -> intake.setLiftLocation(CollectorSubsystem.LiftState.RAISED))),
                new InstantCommand(() -> drive.adjustPose(new Pose2d(10, 0, 0)))
        ));
    }
}
