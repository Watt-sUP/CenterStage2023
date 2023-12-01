package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.subsystems.ApriltagSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;

@Config
@TeleOp(name = "Vision: AprilTag Preview", group = "Vision")
public class AlignWithAprilTagSample extends LinearOpMode {

    public static int TARGET_ID = 1;
    public static ApriltagSubsystem.Pose TARGET_POSITION = new ApriltagSubsystem.Pose(5, 0, 0), THRESHOLDS = new ApriltagSubsystem.Pose(2, 2, 2.5);
    private RobotStates robotState = RobotStates.DRIVE;

    @Override
    public void runOpMode() throws InterruptedException {

        OdometrySubsystem odometry = new OdometrySubsystem(
                new SimpleServo(hardwareMap, "odo_left", 0, 300),
                new SimpleServo(hardwareMap, "odo_right", 0, 300),
                new SimpleServo(hardwareMap, "odo_back", 0, 1800)
        );
        ApriltagSubsystem apriltagSubsystem = new ApriltagSubsystem(hardwareMap, "Webcam 1", TARGET_ID);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());
        odometry.raise();

        while (opModeInInit()) {
            if (isStopRequested())
                return;

            List<ApriltagSubsystem.Pose> detections = apriltagSubsystem.getDetections();
            for (ApriltagSubsystem.Pose detection : detections) {
                telemetry.addData("Forward Offset (Inch)", detection.y);
                telemetry.addData("Strafe Offset (Inch)", detection.x);
                telemetry.addData("Turn Offset (Degrees)", detection.heading);
                telemetry.addData("Detection ID", detection.id);

                telemetry.update();
            }
        }

        while (opModeIsActive()) {
            if (isStopRequested())
                return;

            switch (robotState) {
                case DRIVE:
                    // Driver controls
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x,
                                    -gamepad1.right_stick_x
                            )
                    );

                    if (gamepad1.y) {
                        // Shut down the motors, prepare the odometry and start the camera processing
                        drive.setMotorPowers(0, 0, 0, 0);
                        odometry.lower();

                        if (apriltagSubsystem.portal.getCameraState() != VisionPortal.CameraState.STREAMING)
                            apriltagSubsystem.portal.resumeStreaming();
                        robotState = RobotStates.ALIGN_TO_TAG;
                    }
                    break;

                case ALIGN_TO_TAG:
                    // Don't do anything if the camera hasn't started yet
                    if (apriltagSubsystem.portal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                        ApriltagSubsystem.Pose adjustment;
                        List<ApriltagSubsystem.Pose> detections = apriltagSubsystem.getDetections();

                        for (ApriltagSubsystem.Pose detection : detections) {
                            // Log tag position
                            telemetry.addLine("Tag Position");
                            telemetry.addLine("--------------------------");
                            telemetry.addData("Forward Offset", detection.y);
                            telemetry.addData("Strafe Offset", detection.x);
                            telemetry.addData("Turn Offset", detection.heading);

                            // Bearing is positive to the left and Y increases as you stray further away,
                            // leaving X to be reversed
                            adjustment = detection.minus(TARGET_POSITION);
                            adjustment.x *= -1;

                            // Don't start movement until existing trajectories finish or you're already in position
                            if (!drive.isBusy())
                                drive.adjustPoseAsync(adjustment.toPose2d());
                        }
                    }

                    if (gamepad1.b) {
                        if (apriltagSubsystem.portal.getCameraState() == VisionPortal.CameraState.STREAMING)
                            apriltagSubsystem.portal.stopStreaming();

                        // End any ongoing trajectory
                        if (drive.isBusy())
                            drive.breakFollowing();

                        // Lift the odometry for proper driving
                        odometry.raise();
                        robotState = RobotStates.DRIVE;
                    }

                    telemetry.addData("Target Pose", TARGET_POSITION.y + ", " +
                            TARGET_POSITION.x + ", " + TARGET_POSITION.heading);
                    drive.update();
                    break;

                default:
                    if (apriltagSubsystem.portal.getCameraState() == VisionPortal.CameraState.STREAMING)
                        apriltagSubsystem.portal.stopStreaming();

                    odometry.raise();
                    robotState = RobotStates.DRIVE;
            }

            telemetry.addData("Current Mode", robotState.toString());
            telemetry.update();
        }
        apriltagSubsystem.shutdown();
    }

    private enum RobotStates {
        ALIGN_TO_TAG,
        DRIVE
    }
}