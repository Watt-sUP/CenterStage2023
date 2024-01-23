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

    public static int TARGET_ID = 9;
    private RobotStates robotState = RobotStates.DRIVE;
    public static Pose2d TARGET_POSE = new Pose2d(15, 0, 0);
    private List<Pose2d> tagPoses;

    @Override
    public void runOpMode() throws InterruptedException {

        OdometrySubsystem odometry = new OdometrySubsystem(
                new SimpleServo(hardwareMap, "odo_left", 0, 180),
                new SimpleServo(hardwareMap, "odo_right", 0, 180),
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

            tagPoses = apriltagSubsystem.getDetectionPoses();
            for (Pose2d pose : tagPoses) {
                telemetry.addData("Forward Offset (Inch)", pose.getX());
                telemetry.addData("Strafe Offset (Inch)", pose.getY());
                telemetry.addData("Turn Offset (Degrees)", Math.toDegrees(pose.getHeading()));

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

                    tagPoses = apriltagSubsystem.getDetectionPoses();
                    for (Pose2d pose : tagPoses) {
                        telemetry.addData("Forward Offset (Inch)", pose.getX());
                        telemetry.addData("Strafe Offset (Inch)", pose.getY());
                        telemetry.addData("Turn Offset (Degrees)", Math.toDegrees(pose.getHeading()));

                        telemetry.addData("Suggested Adjustment", pose.minus(TARGET_POSE));
                        telemetry.update();
                    }

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
                    if (gamepad1.b) {
                        // Lift the odometry for proper driving
                        odometry.raise();
                        robotState = RobotStates.DRIVE;
                    }

                    // Don't check for more tags if you're already following a trajectory
                    if (drive.isBusy()) {
                        drive.update();
                        continue;
                    }

                    sleep(100); // Share the CPU
                    // Don't do anything if the camera hasn't started yet
                    if (apriltagSubsystem.portal.getCameraState() == VisionPortal.CameraState.STREAMING) {

                        tagPoses = apriltagSubsystem.getDetectionPoses();
                        for (Pose2d pose : tagPoses) {

                            // Log tag position
                            telemetry.addLine("Tag Position");
                            telemetry.addLine("--------------------------");
                            telemetry.addData("Forward Offset", pose.getX());
                            telemetry.addData("Strafe Offset", pose.getY());
                            telemetry.addData("Turn Offset", pose.getHeading());

                            Pose2d adjustment = pose.minus(TARGET_POSE);
                            drive.setPoseEstimate(new Pose2d(0, 0, 0));
                            drive.lineToPose(adjustment);
                            robotState = RobotStates.DRIVE;
                        }
                    }
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