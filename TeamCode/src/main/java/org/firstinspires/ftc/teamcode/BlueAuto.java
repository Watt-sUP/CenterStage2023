package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.commands.subsystems.CollectorSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.TeamPropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.HashMap;

@Autonomous(name = "Blue Autonomous", group = "auto")
public class BlueAuto extends CommandOpMode {
    @Override
    public void initialize() {
        TeamPropProcessor processor = new TeamPropProcessor(TeamPropProcessor.PropColor.BLUE, 4000., 100., 450.);
        TeamPropProcessor.Location location = TeamPropProcessor.Location.NOT_FOUND;

        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .addProcessor(processor)
                .setAutoStopLiveView(true)
                .build();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        telemetry.addLine("Loading trajectory...");
        telemetry.update();

        OdometrySubsystem odometrySystem = new OdometrySubsystem(
                new SimpleServo(hardwareMap, "odo_left", 0, 300),
                new SimpleServo(hardwareMap, "odo_right", 0, 300),
                new SimpleServo(hardwareMap, "odo_back", 0, 1800)
        );
        CollectorSubsystem collectorSystem = new CollectorSubsystem(
                new SimpleServo(hardwareMap, "v4b_left", 0, 180),
                new SimpleServo(hardwareMap, "v4b_right", 0, 180),
                new SimpleServo(hardwareMap, "claw", 0, 300),
                new SimpleServo(hardwareMap, "claw_r", 0, 180)
        );

        TrajectorySequence lines = drive.trajectorySequenceBuilder(new Pose2d(-63.88, 12.74, Math.toRadians(0.00)))
                .forward(47)
                .lineToConstantHeading(new Vector2d(-37.53, 11.61))
                .build();
        TrajectorySequence rightPositioning = drive.trajectorySequenceBuilder(lines.end())
                .forward(3)
                .strafeLeft(5)
                .build();
        TrajectorySequence leftPositioning = drive.trajectorySequenceBuilder(lines.end())
                .forward(3)
                .build();
        TrajectorySequence park = drive.trajectorySequenceBuilder(leftPositioning.end().plus(new Pose2d(0, 0, 90)))
                .lineToLinearHeading(new Pose2d(-58.28, 11.28, Math.toRadians(90)))
                .splineTo(new Vector2d(-60.80, 61.00), Math.toRadians(90.00))
                .build();
        Trajectory park_reverse = drive.trajectoryBuilder(rightPositioning.end().plus(new Pose2d(0, 0, -90)), true)
                .splineTo(new Vector2d(-60.80, 61.00), Math.toRadians(90.00))
                .build();

        drive.setPoseEstimate(lines.start());
        odometrySystem.lower();

        telemetry.addLine("Ready!");
        telemetry.update();

        while (!isStarted()) {
            if (isStopRequested())
                return;

            location = processor.getCurrentLocation();
            telemetry.addData("FPS", visionPortal.getFps());
            telemetry.addData("Current Location", location);
            telemetry.addData("Contour Area", processor.getLargestContourArea());
            telemetry.addData("Contour Coords", processor.getLargestContourCoords().toString());
            telemetry.update();
        }

        waitForStart();
        TeamPropProcessor.Location finalLocation = location;
        schedule(new SequentialCommandGroup(
                new InstantCommand(() -> drive.followTrajectorySequence(lines)),
                new WaitCommand(250),
                new InstantCommand(() -> collectorSystem.setLiftLocation(CollectorSubsystem.LiftState.STACK)),
                new SelectCommand(
                        new HashMap<Object, Command>() {{
                            put(TeamPropProcessor.Location.RIGHT, new SequentialCommandGroup(
                                    new InstantCommand(() -> drive.followTrajectorySequence(rightPositioning)),
                                    new InstantCommand(() -> drive.turn(Math.toRadians(-90))),
                                    new WaitCommand(300),
                                    new InstantCommand(collectorSystem::toggleLiftLocation),
                                    new WaitCommand(400),
                                    new InstantCommand(collectorSystem::toggleLiftLocation),
                                    new WaitCommand(250),
                                    new InstantCommand(() -> drive.followTrajectory(park_reverse))
                            ));
                            put(TeamPropProcessor.Location.NOT_FOUND, new SequentialCommandGroup(
                                    new InstantCommand(() ->
                                    {
                                        drive.followTrajectorySequence(leftPositioning);
                                        drive.turn(Math.toRadians(90));
                                    }),
                                    new WaitCommand(300),
                                    new InstantCommand(collectorSystem::toggleLiftLocation),
                                    new WaitCommand(400),
                                    new InstantCommand(collectorSystem::toggleLiftLocation),
                                    new WaitCommand(250),
                                    new InstantCommand(() -> drive.followTrajectorySequence(park))
                            ));
                            put(TeamPropProcessor.Location.LEFT, new SequentialCommandGroup(
                                    new InstantCommand(() -> {
                                        drive.followTrajectorySequence(leftPositioning);
                                        drive.turn(Math.toRadians(90));
                                    }),
                                    new WaitCommand(300),
                                    new InstantCommand(collectorSystem::toggleLiftLocation),
                                    new WaitCommand(400),
                                    new InstantCommand(collectorSystem::toggleLiftLocation),
                                    new WaitCommand(250),
                                    new InstantCommand(() -> drive.followTrajectorySequence(park))
                            ));
                            put(TeamPropProcessor.Location.MIDDLE, new SequentialCommandGroup(
                                    new WaitCommand(300),
                                    new InstantCommand(collectorSystem::toggleLiftLocation),
                                    new WaitCommand(400),
                                    new InstantCommand(collectorSystem::toggleLiftLocation),
                                    new WaitCommand(250),
                                    new InstantCommand(() -> drive.turn(Math.toRadians(-90))),
                                    new InstantCommand(() -> drive.followTrajectory(park_reverse))
                            ));
                        }},
                        () -> finalLocation
                )
        ));
    }
}