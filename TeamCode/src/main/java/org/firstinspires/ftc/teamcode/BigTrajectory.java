package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Path Test")
public class BigTrajectory extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        telemetry.addLine("Loading trajectory...");
        telemetry.update();

        TrajectorySequence path = drive.trajectorySequenceBuilder(new Pose2d(-64.07, -40.29, Math.toRadians(0.00)))
                .splineTo(new Vector2d(-11.42, -58.41), Math.toRadians(270.00))
                .lineToConstantHeading(new Vector2d(-11.80, 34.07))
                .splineTo(new Vector2d(-62.37, 12.36), Math.toRadians(180.00))
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(-64.07, -40.29))
                .build();

        drive.setPoseEstimate(path.start());

        telemetry.addLine("Ready!");
        telemetry.update();

        while (!isStarted())
            if (isStopRequested())
                return;

        drive.followTrajectorySequence(path);
    }
}
