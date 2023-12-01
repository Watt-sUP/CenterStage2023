package org.firstinspires.ftc.teamcode.commands.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ApriltagSubsystem extends SubsystemBase {

    private final AprilTagProcessor aprilTagProcessor;
    public VisionPortal portal;
    private List<Integer> targetsList;

    public ApriltagSubsystem(VisionPortal portal, AprilTagProcessor processor) {
        aprilTagProcessor = processor;
        this.portal = portal;
    }

    public ApriltagSubsystem(HardwareMap hardwareMap, String cameraName) {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .build();

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, cameraName))
                .addProcessor(aprilTagProcessor)
                .setAutoStopLiveView(true)
                .build();
    }

    public ApriltagSubsystem(VisionPortal portal, AprilTagProcessor processor, Integer... targets) {
        this(portal, processor);
        this.setTargetTags(targets);
    }

    public ApriltagSubsystem(HardwareMap hardwareMap, String cameraName, Integer... targets) {
        this(hardwareMap, cameraName);
        this.setTargetTags(targets);
    }

    public void setTargetTags(Integer... targets) {
        targetsList = Arrays.asList(targets);
    }

    public List<Pose> getDetections() {
        if (portal.getCameraState() != VisionPortal.CameraState.STREAMING || targetsList.isEmpty())
            return new ArrayList<>();

        List<Pose> detections = new ArrayList<>();
        List<AprilTagDetection> aprilTagDetections = aprilTagProcessor.getDetections();
        for (AprilTagDetection detection : aprilTagDetections) {
            if (targetsList.contains(detection.id) && detection.metadata != null)
                detections.add(new Pose(
                        detection.id,
                        detection.ftcPose.x,
                        detection.ftcPose.y,
                        detection.ftcPose.yaw
                ));
            else if (targetsList.contains(detection.id)) {
                detections.add(new Pose(
                        detection.id,
                        -1, -1, -1
                ));
            }
        }

        return detections;
    }

    public void shutdown() {
        if (portal.getCameraState() == VisionPortal.CameraState.CAMERA_DEVICE_CLOSED)
            return;

        portal.close();
    }

    public static class Pose {
        public double x, y, heading;
        public int id;

        public Pose(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }

        public Pose(int id, double x, double y, double heading) {
            this(x, y, heading);
            this.id = id;
        }

        public Pose plus(Pose pose) {
            return new Pose(this.x + pose.x, this.y + pose.y, this.heading + pose.heading);
        }

        public Pose times(double scalar) {
            return new Pose(this.x * scalar, this.y * scalar, this.heading * scalar);
        }

        public Pose minus(Pose pose) {
            return this.plus(pose.times(-1));
        }

        public Pose2d toPose2d() {
            return new Pose2d(this.x, this.y, this.heading);
        }
    }
}
