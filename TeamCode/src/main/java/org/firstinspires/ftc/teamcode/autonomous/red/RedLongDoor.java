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

import java.util.HashMap;
import java.util.Locale;
import java.util.Map;

@Autonomous(name = "Red Long (Stage Door)", group = "Auto (Long)")
public class RedLongDoor extends CommandOpMode {

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

        OdometrySubsystem odometry = new OdometrySubsystem(this);
        CollectorSubsystem intake = new CollectorSubsystem(hardwareMap);
        DepositSubsystem outtake = new DepositSubsystem(hardwareMap);

        TrajectorySequence leftPurple = drive.trajectorySequenceBuilder(generator.getStartingPose())
                .splineToSplineHeading(new Pose2d(
                        new Vector2d(-47.5, -31).minus(Vector2d.polar(13, Math.toRadians(180))),
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

        TrajectorySequence whiteLeft = drive.trajectorySequenceBuilder(leftPurple.end(), 40)
                .setTangent(Math.toRadians(90.00))
                .splineToSplineHeading(new Pose2d(-51.50, -12, Math.toRadians(180)), Math.toRadians(180))
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
                .lineToLinearHeading(new Pose2d(-56.50, -12, Math.toRadians(180.00)))
                .addTemporalMarker(intake::toggleClamp)
                .waitSeconds(0.5)
                .build();
        TrajectorySequence whiteMiddle = drive.trajectorySequenceBuilder(middlePurple.end(), 40)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-51.50, -35.75, Math.toRadians(180.00)), Math.toRadians(180))
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
                .lineToLinearHeading(new Pose2d(-56.50, -35.75, Math.toRadians(180.00)))
                .addTemporalMarker(intake::toggleClamp)
                .waitSeconds(0.5)
                .build();
        TrajectorySequence whiteRight = drive.trajectorySequenceBuilder(rightPurple.end(), 40)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-51.50, -35.75, Math.toRadians(180.00)), Math.toRadians(180))
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
                .lineToLinearHeading(new Pose2d(-56.50, -35.75, Math.toRadians(180.00)))
                .addTemporalMarker(intake::toggleClamp)
                .waitSeconds(0.5)
                .build();

        Map<PropLocations, TrajectorySequence> backdropsWhite = new HashMap<PropLocations, TrajectorySequence>() {{
            put(PropLocations.LEFT, drive.trajectorySequenceBuilder(whiteLeft.end(), 50)
                    .setReversed(true)
                    .splineTo(new Pose2d(-24, -11, Math.toRadians(0.00)))
                    .splineTo(new Pose2d(24, -11, Math.toRadians(0.00)))
                    .splineTo(new Vector2d(50.50, -35.50), Math.toRadians(0.00))
                    .build()
            );
            put(PropLocations.MIDDLE,
                    drive.trajectorySequenceBuilder(whiteMiddle.end(), 50)
                            .setReversed(true)
                            .splineTo(new Vector2d(24.00, -35.50), Math.toRadians(0.00))
                            .splineTo(new Vector2d(50.50, -30.50), Math.toRadians(0.00))
                            .build()
            );
            put(PropLocations.RIGHT,
                    drive.trajectorySequenceBuilder(whiteRight.end(), 50)
                            .setReversed(true)
                            .splineTo(new Vector2d(-24, -11), Math.toRadians(0.00))
                            .splineTo(new Vector2d(24, -11), Math.toRadians(0.00))
                            .splineTo(new Vector2d(50.50, -30.50), Math.toRadians(0.00))
                            .build()
            );
        }};

        Map<PropLocations, Vector2d> yellowLocation = new HashMap<PropLocations, Vector2d>() {{
            put(PropLocations.LEFT, new Vector2d(50.50, -30.50));
            put(PropLocations.MIDDLE, new Vector2d(50.50, -35.50));
            put(PropLocations.RIGHT, new Vector2d(50.50, -42.50));
        }};

        TrajectorySequence stackLeft = drive.trajectorySequenceBuilder(new Pose2d(yellowLocation.get(PropLocations.LEFT), Math.PI), 50)
                .splineTo(new Vector2d(18, -12), Math.toRadians(180))
                .waitSeconds(.25)
                .lineToLinearHeading(new Pose2d(-56.50, -12, Math.toRadians(180)))
                .build();
        TrajectorySequence stackRight = drive.trajectorySequenceBuilder(new Pose2d(yellowLocation.get(PropLocations.RIGHT), Math.PI), 50)
                .splineTo(new Vector2d(18, -12), Math.toRadians(180))
                .waitSeconds(.25)
                .lineToLinearHeading(new Pose2d(-56.50, -12, Math.toRadians(180)))
                .build();
        TrajectorySequence stackMiddle = drive.trajectorySequenceBuilder(new Pose2d(yellowLocation.get(PropLocations.MIDDLE), Math.PI), 50)
                .splineTo(new Vector2d(18, -12), Math.toRadians(180))
                .waitSeconds(.25)
                .lineToLinearHeading(new Pose2d(-56.50, -12, Math.toRadians(180)))
                .build();

        TrajectorySequence backdropCycleLeft = drive.trajectorySequenceBuilder(stackLeft.end(), 50)
                .setReversed(true)
                .splineTo(new Pose2d(24, -11, Math.toRadians(0.00)))
                .splineTo(new Vector2d(50.50, -30.50), Math.toRadians(0.00))
                .build();
        TrajectorySequence backdropCycleMiddle = drive.trajectorySequenceBuilder(stackMiddle.end(), 50)
                .setReversed(true)
                .splineTo(new Pose2d(24, -11, Math.toRadians(0.00)))
                .splineTo(new Vector2d(50.50, -30.50), Math.toRadians(0.00))
                .build();
        TrajectorySequence backdropCycleRight = drive.trajectorySequenceBuilder(stackRight.end(), 50)
                .setReversed(true)
                .splineTo(new Pose2d(24, -11, Math.toRadians(0.00)))
                .splineTo(new Vector2d(50.50, -30.50), Math.toRadians(0.00))
                .build();


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
                    intake.setLiftLocation(CollectorSubsystem.LiftState.STACK);
                    intake.adjustLiftPosition(10.0);
                }),
                new RunByCaseCommand(location.toString(), drive, leftPurple, middlePurple, rightPurple, true),
                new InstantCommand(intake::toggleLiftLocation).andThen(
                        new InstantCommand(() -> intake.setLiftLocation(CollectorSubsystem.LiftState.STACK)),
                        new WaitCommand(200),
                        new InstantCommand(() -> intake.setLiftLocation(CollectorSubsystem.LiftState.RAISED))
                ),
                new RunByCaseCommand(location.toString(), drive, whiteLeft, whiteMiddle, whiteRight, true),
                new InstantCommand(() -> drive.followTrajectorySequenceAsync(backdropsWhite.get(location))),
                new ParallelCommandGroup(
                        new RunCommand(drive::update).interruptOn(() -> !drive.isBusy()),
                        new WaitCommand(700).andThen(new InstantCommand(intake::toggleClamp)),
                        new WaitUntilCommand(() -> drive.getPoseEstimate().getX() > 0)
                                .andThen(
                                        new InstantCommand(() -> intake.setLiftLocation(CollectorSubsystem.LiftState.STACK)),
                                        new WaitCommand(300),
                                        new InstantCommand(() -> {
                                            intake.setClampPosition(25);
                                            outtake.toggleBlockers();
                                            outtake.toggleSpike();

                                            if (location == PropLocations.LEFT)
                                                intake.adjustLiftPosition(5.0);
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
                new ParallelCommandGroup(
                        new RunByCaseCommand(location.toString(), drive, backdropCycleLeft, backdropCycleMiddle, backdropCycleRight, false),
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
                new InstantCommand(() -> drive.lineToPose(new Pose2d(48, -12, Math.toRadians(180))))
                        .andThen(new InstantCommand(() -> intake.setLiftLocation(CollectorSubsystem.LiftState.RAISED)))
        ));
    }
}
