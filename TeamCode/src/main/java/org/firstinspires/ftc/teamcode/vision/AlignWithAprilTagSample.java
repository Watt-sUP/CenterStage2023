package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import androidx.annotation.NonNull;

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

import java.util.List;

@Config
@TeleOp(name = "Vision: AprilTag Preview", group = "Vision")
public class AlignWithAprilTagSample extends LinearOpMode {

    public static int TARGET_ID = 1;
    public static Pose TARGET_POSITION = new Pose(5, 0, 0), THRESHOLDS = new Pose(2, 2, 2.5);
    private RobotStates robotState = RobotStates.DRIVE;

    @Override
    public void runOpMode() throws InterruptedException {

        OdometrySubsystem odometry = new OdometrySubsystem(
                new SimpleServo(hardwareMap, "odo_left", 0, 300),
                new SimpleServo(hardwareMap, "odo_right", 0, 300),
                new SimpleServo(hardwareMap, "odo_back", 0, 1800)
        );

        AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
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
        aprilTagProcessor.setDecimation(3);
        visionPortal.stopLiveView();
        odometry.raise();

        while (opModeInInit()) {
            if (isStopRequested())
                return;

            List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
            for (AprilTagDetection detection : detections)
                if (detection.metadata != null && detection.id == TARGET_ID) {
                    Pose positionData = getAprilTagPose(detection);
                    telemetry.addData("Forward Offset (CM)", positionData.f);
                    telemetry.addData("Strafe Offset (CM)", positionData.s);
                    telemetry.addData("Turn Offset (Degrees)", positionData.r);

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

                        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)
                            visionPortal.resumeStreaming();
                        robotState = RobotStates.ALIGN_TO_TAG;
                    }
                    break;

                case ALIGN_TO_TAG:
                    // Don't do anything if the camera hasn't started yet
                    if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                        Pose adjustment;
                        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

                        for (AprilTagDetection detection : detections)
                            if (detection.metadata != null && detection.id == TARGET_ID) {
                                Pose tagPosition = getAprilTagPose(detection);
                                // Log tag position
                                telemetry.addLine("Tag Position");
                                telemetry.addLine("--------------------------");
                                telemetry.addData("Forward Offset", tagPosition.f);
                                telemetry.addData("Strafe Offset", tagPosition.s);
                                telemetry.addData("Turn Offset", tagPosition.r);

                                // Bearing is positive to the left and Y increases as you stray further away,
                                // leaving X to be reversed
                                adjustment = tagPosition.minus(TARGET_POSITION);
                                adjustment.s *= -1;

                                // Don't start movement until existing trajectories finish or you're already in position
                                if (!adjustment.fitsThreshold(THRESHOLDS) && !drive.isBusy())
                                    drive.adjustPoseAsync(adjustment.toPose2d());
                            }
                    }

                    if (gamepad1.b) {
                        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING)
                            visionPortal.stopStreaming();

                        // End any ongoing trajectory
                        if (drive.isBusy())
                            drive.breakFollowing();

                        // Lift the odometry for proper driving
                        odometry.raise();
                        robotState = RobotStates.DRIVE;
                    }

                    telemetry.addData("Target Pose", TARGET_POSITION.f + ", " +
                            TARGET_POSITION.s + ", " + TARGET_POSITION.r);
                    drive.update();
                    break;

                default:
                    if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING)
                        visionPortal.stopStreaming();

                    odometry.raise();
                    robotState = RobotStates.DRIVE;
            }

            telemetry.addData("Current Mode", robotState.toString());
            telemetry.update();
        }
        visionPortal.close();
    }

    @NonNull
    public Pose getAprilTagPose(AprilTagDetection detection) {
        return new Pose(
                detection.ftcPose.y,
                detection.ftcPose.x,
                detection.ftcPose.bearing
        );
    }

    private enum RobotStates {
        ALIGN_TO_TAG,
        DRIVE
    }
}

class Pose {
    public double f, s, r;

    public Pose(double f, double s, double r) {
        this.f = f;
        this.s = s;
        this.r = r;
    }

    public Pose plus(Pose x) {
        return new Pose(this.f + x.f, this.s + x.s, this.r + x.r);
    }

    public Pose times(double scalar) {
        return new Pose(this.f * scalar, this.s * scalar, this.r * scalar);
    }

    public Pose minus(Pose x) {
        return this.plus(x.times(-1));
    }

    public boolean fitsThreshold(Pose threshold) {
        return Math.abs(this.f) <= threshold.f &&
                Math.abs(this.s) <= threshold.s &&
                Math.abs(this.r) <= threshold.r;
    }

    public Pose2d toPose2d() {
        return new Pose2d(this.f, this.s, this.r);
    }
}