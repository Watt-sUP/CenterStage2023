package org.firstinspires.ftc.teamcode.autonomous.blue;

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
import org.firstinspires.ftc.teamcode.autonomous.PathGenerator;
import org.firstinspires.ftc.teamcode.autonomous.assets.AllianceColor;
import org.firstinspires.ftc.teamcode.autonomous.assets.PropLocations;
import org.firstinspires.ftc.teamcode.autonomous.assets.StartingPosition;
import org.firstinspires.ftc.teamcode.commands.RunByCaseCommand;
import org.firstinspires.ftc.teamcode.commands.subsystems.CollectorSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.TensorflowSubsystem;
import org.firstinspires.ftc.teamcode.roadrunner.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Locale;
import java.util.Map;
import java.util.stream.Collectors;

@Autonomous(name = "Blue Long (Side)", group = "Auto (Long)")
public class BlueLongSide extends CommandOpMode {

    private PropLocations location;
    private SampleMecanumDrive drive;

    @Override
    public void initialize() {

        TensorflowSubsystem tensorflow = new TensorflowSubsystem(hardwareMap, "Webcam 1",
                "blue_prop.tflite", "Blue Prop");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);
        PathGenerator generator = new PathGenerator(drive);

        generator.setStartingLocation(AllianceColor.BLUE, StartingPosition.AUDIENCE);
        tensorflow.setMinConfidence(0.8);

        telemetry.addLine("Loading trajectories...");
        telemetry.update();

        OdometrySubsystem odometry = new OdometrySubsystem(this);
        CollectorSubsystem intake = new CollectorSubsystem(hardwareMap);
        DepositSubsystem outtake = new DepositSubsystem(hardwareMap);

        TrajectorySequence rightPurple = drive.trajectorySequenceBuilder(generator.getStartingPose())
                .splineTo(new Vector2d(-46, 39), Math.toRadians(-90.00))
                .build();

        TrajectorySequence middlePurple = drive.trajectorySequenceBuilder(generator.getStartingPose())
                .splineTo(new Vector2d(-39.00, 38.00), Math.toRadians(-90.00))
                .build();

        TrajectorySequence leftPurple = drive.trajectorySequenceBuilder(generator.getStartingPose())
                .splineTo(new Vector2d(-24.5, 33)
                        .minus(Vector2d.polar(12, Math.toRadians(-30))), Math.toRadians(-30.00))
                .build();

        Map<PropLocations, TrajectorySequence> whites = Arrays.stream(PropLocations.values())
                .collect(Collectors.toMap(
                        location -> location,
                        location -> {
                            Map<PropLocations, Pose2d> endPoses = new HashMap<PropLocations, Pose2d>() {{
                                put(PropLocations.LEFT, leftPurple.end());
                                put(PropLocations.MIDDLE, middlePurple.end());
                                put(PropLocations.RIGHT, rightPurple.end());
                            }};

                            return drive.trajectorySequenceBuilder(endPoses.get(location), 40)
                                    .setTangent(Math.toRadians(180))
                                    .splineToLinearHeading(new Pose2d(-52.50, 35.75, Math.toRadians(180.00)), Math.toRadians(180))
                                    .addTemporalMarker(() -> {
                                        intake.setLiftLocation(CollectorSubsystem.LiftState.STACK);
                                        intake.adjustLiftPosition(-5.0);
                                    })
                                    .waitSeconds(0.3)
                                    .addTemporalMarker(intake::toggleClamp)
                                    .setConstraints(
                                            SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(30)
                                    )
                                    .lineToLinearHeading(new Pose2d(-56.50, 35.75, Math.toRadians(180.00)))
                                    .addTemporalMarker(intake::toggleClamp)
                                    .waitSeconds(0.5)
                                    .build();
                        }
                ));

        Map<PropLocations, TrajectorySequence> backdrops = Arrays.stream(PropLocations.values())
                .collect(Collectors.toMap(
                        location -> location,
                        location -> drive.trajectorySequenceBuilder(whites.get(location).end(), 50)
                                .setReversed(true)
                                .splineTo(new Vector2d(-24.00, 60.00), Math.toRadians(0.00))
                                .splineTo(new Vector2d(4.00, 60.00), Math.toRadians(0.00))
                                .splineTo(new Vector2d(50.50, (location != PropLocations.LEFT) ? 42.50 : 35.50), Math.toRadians(0.00))
                                .build()
                ));
        Map<PropLocations, Vector2d> yellowLocation = new HashMap<PropLocations, Vector2d>() {{
            put(PropLocations.RIGHT, new Vector2d(50.50, 30.00));
            put(PropLocations.MIDDLE, new Vector2d(50.50, 35.50));
            put(PropLocations.LEFT, new Vector2d(50.50, 42.50));
        }};

        TrajectorySequence stackRight = drive.trajectorySequenceBuilder(new Pose2d(yellowLocation.get(PropLocations.RIGHT), Math.PI), 50)
                .splineTo(new Vector2d(7.00, 60.00), Math.toRadians(180.00))
                .splineTo(new Vector2d(-37.00, 60.00), Math.toRadians(180.00))
                .setConstraints(
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(45)
                )
                .lineToLinearHeading(new Pose2d(-50.00, 36, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-58.50, 36, Math.toRadians(180)))
                .build();
        TrajectorySequence stackMid = drive.trajectorySequenceBuilder(new Pose2d(yellowLocation.get(PropLocations.MIDDLE), Math.PI), 50)
                .splineTo(new Vector2d(7.00, 60.00), Math.toRadians(180.00))
                .splineTo(new Vector2d(-37.00, 60.00), Math.toRadians(180.00))
                .setConstraints(
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(35)
                )
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-50.00, 36, Math.toRadians(180)), Math.toRadians(180.00))
                .lineToLinearHeading(new Pose2d(-56.75, 36, Math.toRadians(180)))
                .build();
        TrajectorySequence stackLeft = drive.trajectorySequenceBuilder(new Pose2d(yellowLocation.get(PropLocations.LEFT), Math.PI), 50)
                .splineTo(new Vector2d(7.00, 60.00), Math.toRadians(180.00))
                .splineTo(new Vector2d(-37.00, 60.00), Math.toRadians(180.00))
                .setConstraints(
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(35)
                )
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-50.00, 36, Math.toRadians(180)), Math.toRadians(180.00))
                .lineToLinearHeading(new Pose2d(-56.75, 36, Math.toRadians(180)))
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

        tensorflow.shutdown();
        schedule(new SequentialCommandGroup(
                new InstantCommand(() -> {
                    intake.setLiftLocation(CollectorSubsystem.LiftState.STACK);
                    intake.adjustLiftPosition(10.0);
                }),
                new RunByCaseCommand(location.toString(), drive, leftPurple, middlePurple, rightPurple, true),
                new InstantCommand(intake::toggleLiftLocation).andThen(
                        new InstantCommand(() -> intake.setLiftLocation(CollectorSubsystem.LiftState.STACK)),
                        new WaitCommand(200),
                        new InstantCommand(() -> intake.setLiftLocation(CollectorSubsystem.LiftState.RAISED))
                ),
                new InstantCommand(() -> drive.followTrajectorySequence(whites.get(location))),
                new InstantCommand(() -> drive.followTrajectorySequenceAsync(backdrops.get(location))),
                new ParallelCommandGroup(
                        new RunCommand(drive::update).interruptOn(() -> !drive.isBusy()),
                        new WaitCommand(700).andThen(new InstantCommand(intake::toggleClamp)),
                        new WaitUntilCommand(() -> drive.getPoseEstimate().getX() > 0)
                                .andThen(
                                        new InstantCommand(() -> intake.setLiftLocation(CollectorSubsystem.LiftState.STACK)),
                                        new WaitCommand(300),
                                        new InstantCommand(() -> {
                                            intake.setClampPosition(25);
                                            intake.adjustLiftPosition(5.0);
                                            outtake.toggleBlockers();
                                            outtake.toggleSpike();
                                        })
                                )
                ),
                new InstantCommand(outtake::toggleBlockers).andThen(
                        new WaitCommand(300),
                        new InstantCommand(outtake::toggleSpike),
                        new WaitCommand(300),
                        new InstantCommand(outtake::toggleSpike)
                ),
                new InstantCommand(() -> drive.lineToPose(new Pose2d(yellowLocation.get(location), Math.toRadians(180.00))))
                        .andThen(
                                new InstantCommand(outtake::toggleBlockers),
                                new WaitCommand(300),
                                new InstantCommand(outtake::toggleSpike),
                                new WaitCommand(300),
                                new InstantCommand(() -> outtake.setSlidesPosition(0))
                        ),
                new RunByCaseCommand(location.toString(), drive, stackLeft, stackMid, stackRight, true)
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
                                new WaitCommand(300),
                                new InstantCommand(outtake::toggleSpike),
                                new WaitCommand(300),
                                new InstantCommand(outtake::toggleSpike),
                                new WaitCommand(300)
                        ),
                new InstantCommand(outtake::toggleBlockers)
                        .andThen(
                                new WaitCommand(300),
                                new InstantCommand(outtake::toggleSpike),
                                new InstantCommand(() -> outtake.setSlidesPosition(0))
                        ),

                new InstantCommand(() -> drive.adjustPose(new Pose2d(-5, 0, 0))),
                new InstantCommand(() -> drive.lineToPose(new Pose2d(48, 60, Math.toRadians(180))))
                        .andThen(new InstantCommand(() -> intake.setLiftLocation(CollectorSubsystem.LiftState.RAISED)))
        ));
    }

    @Override
    public void run() {
        super.run();

        if (!drive.isBusy())
            drive.updatePoseEstimate();
    }
}
