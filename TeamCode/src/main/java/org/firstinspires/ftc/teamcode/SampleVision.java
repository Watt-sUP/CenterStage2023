package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

@Config
@TeleOp(name = "Vision: AprilTag Preview", group = "Vision")
public class SampleVision extends LinearOpMode {

    public static int TARGET_ID = 1;
    public static double FORWARD_THRESHOLD = 15, STRAFE_THRESHOLD = 5, TURN_THRESHOLD = 5;
    private RobotStates robotState = RobotStates.DRIVE;

    @Override
    public void runOpMode() throws InterruptedException {

        OdometrySubsystem odometrySystem = new OdometrySubsystem(
                new SimpleServo(hardwareMap, "odo_left", 0, 300),
                new SimpleServo(hardwareMap, "odo_right", 0, 300),
                new SimpleServo(hardwareMap, "odo_back", 0, 1800)
        );

        AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.CM, AngleUnit.RADIANS)
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .addProcessor(aprilTagProcessor)
                .setAutoStopLiveView(true)
                .build();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());
        odometrySystem.raise();

        while (opModeInInit()) {
            if (isStopRequested())
                return;

            List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
            for (AprilTagDetection detection : detections) {
                if (detection.metadata != null && detection.id == TARGET_ID) {
                    Map<String, Double> data = getAprilTagPose(detection);
                    telemetry.addData("Forward Offset (CM)", data.get("forward"));
                    telemetry.addData("Strafe Offset (CM)", data.get("strafe"));
                    telemetry.addData("Turn Offset (Degrees)", data.get("turn"));

                    telemetry.update();
                }
            }
        }
        visionPortal.stopLiveView();

        while (opModeIsActive()) {
            switch (robotState) {
                case DRIVE:
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x,
                                    -gamepad1.right_stick_x
                            )
                    );

                    if (gamepad1.y) {
                        drive.setMotorPowers(0, 0, 0, 0);
                        odometrySystem.lower();
                        sleep(200);
                        visionPortal.resumeStreaming();
                        robotState = RobotStates.ALIGN_TO_TAG;
                    }
                    break;

                case ALIGN_TO_TAG:
                    if (gamepad1.b) {
                        visionPortal.stopStreaming();
                        odometrySystem.raise();
                        robotState = RobotStates.DRIVE;
                    }
                    break;

                default:
                    robotState = RobotStates.DRIVE;
                    visionPortal.stopStreaming();
                    odometrySystem.raise();
            }
        }
        visionPortal.close();
    }

    public Map<String, Double> getAprilTagPose(AprilTagDetection detection) {
        Map<String, Double> output = new HashMap<>();
        output.put("turn", detection.ftcPose.bearing);
        output.put("strafe", detection.ftcPose.x);
        output.put("forward", detection.ftcPose.y);

        return output;
    }

    private enum RobotStates {
        ALIGN_TO_TAG,
        DRIVE
    }
}
