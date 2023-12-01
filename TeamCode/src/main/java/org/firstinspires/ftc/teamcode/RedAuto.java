package org.firstinspires.ftc.teamcode;

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

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.commands.subsystems.CollectorSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.TensorflowSubsystem;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.HashMap;

//@Disabled
@Autonomous(name = "Red Autonomous", group = "auto")
public class RedAuto extends CommandOpMode {

    private PropLocations location;

    @Override
    public void initialize() {

        TensorflowSubsystem tensorflow = new TensorflowSubsystem(hardwareMap, "Webcam 1",
                "red_prop.tflite", "Red Prop");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

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
                new SimpleServo(hardwareMap, "stopper_bottom", 0, 1800),
                hardwareMap.dcMotor.get("gli_sus")
        );

        TrajectorySequence rightPurple = drive.trajectorySequenceBuilder(new Pose2d(64.07, 10.85, Math.toRadians(180.00)))
                .splineTo(new Vector2d(22.93, 23.12), Math.toRadians(180.00))
                .setReversed(true)
                .lineTo(new Vector2d(41.99, 22.58))
                .setReversed(false)
                .build();
        Trajectory rightYellow = drive.trajectoryBuilder(rightPurple.end(), true)
                .splineTo(new Vector2d(41.69, 47.35), Math.toRadians(90.00))
                .build();

        TrajectorySequence middlePurple = drive.trajectorySequenceBuilder(new Pose2d(64.07, 10.85, Math.toRadians(180.00)))
                .lineToConstantHeading(new Vector2d(18.94, 10.85))
                .lineToConstantHeading(new Vector2d(40.30, 10.77))
                .build();
        TrajectorySequence middleYellow = drive.trajectorySequenceBuilder(middlePurple.end())
                .setReversed(true)
                .splineTo(new Vector2d(35.85, 47.65), Math.toRadians(90.00))
                .setReversed(false)
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

                if (x < 450)
                    location = PropLocations.MIDDLE;
                else location = PropLocations.RIGHT;
            }

            telemetry.addData("FPS", tensorflow.portal.getFps());
            telemetry.addData("Current Location", location);
            telemetry.update();
        }

        waitForStart();
        schedule(new SequentialCommandGroup(
                new SelectCommand(
                        new HashMap<Object, Command>() {{
                            put(PropLocations.RIGHT, new InstantCommand(() -> drive.followTrajectorySequence(rightPurple)));
                            put(PropLocations.MIDDLE, new InstantCommand(() -> drive.followTrajectorySequence(middlePurple)));
                        }},
                        () -> location
                ),
                new InstantCommand(collectorSystem::toggleLiftLocation),
                new WaitCommand(250),
                new SelectCommand(
                        new HashMap<Object, Command>() {{
                            put(PropLocations.RIGHT, new SequentialCommandGroup(
                                    new InstantCommand(collectorSystem::toggleLiftLocation),
                                    new WaitCommand(600),
                                    new InstantCommand(() -> collectorSystem.setLiftLocation(CollectorSubsystem.LiftState.STACK)),
                                    new WaitCommand(250),
                                    new InstantCommand(() -> drive.followTrajectory(rightYellow))
                            ));
                            put(PropLocations.MIDDLE, new SequentialCommandGroup(
                                    new InstantCommand(collectorSystem::toggleLiftLocation),
                                    new WaitCommand(600),
                                    new InstantCommand(() -> collectorSystem.setLiftLocation(CollectorSubsystem.LiftState.STACK)),
                                    new WaitCommand(250),
                                    new InstantCommand(() -> drive.followTrajectorySequence(middleYellow))
                            ));
                        }},
                        () -> location
                )
        ));
    }

    private enum PropLocations {
        LEFT,
        MIDDLE,
        RIGHT
    }
}
