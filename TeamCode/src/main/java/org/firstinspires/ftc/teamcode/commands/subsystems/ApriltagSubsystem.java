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
import java.util.stream.Collectors;

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
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
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

        List<AprilTagDetection> aprilTagDetections = aprilTagProcessor.getDetections();
        return aprilTagDetections.stream()
                .filter(detection -> targetsList.contains(detection.id))
                .map(detection -> {
                    if (detection.metadata == null)
                        return new Pose(detection.id, -1, -1, -1);

                    double range = detection.ftcPose.range;
                    double x0 = range * Math.cos(detection.ftcPose.bearing - detection.ftcPose.yaw);
                    double y0 = range * Math.sin(detection.ftcPose.bearing - detection.ftcPose.yaw);

                    return new Pose(detection.id, x0, y0, detection.ftcPose.yaw);
                }).collect(Collectors.toList());
    }

    public void shutdown() {
        if (portal.getCameraState() == VisionPortal.CameraState.CAMERA_DEVICE_CLOSED)
            return;

        portal.close();
    }

    public static class Pose {
        public double strafe, forward, heading;
        public int id;

        public Pose(double forward, double strafe, double heading) {
            this.strafe = strafe;
            this.forward = forward;
            this.heading = heading;
        }

        public Pose(int id, double forward, double strafe, double heading) {
            this(forward, strafe, heading);
            this.id = id;
        }

        public Pose plus(Pose pose) {
            return new Pose(this.forward + pose.forward, this.strafe + pose.strafe, this.heading + pose.heading);
        }

        public Pose times(double scalar) {
            return new Pose(this.forward * scalar, this.strafe * scalar, this.heading * scalar);
        }

        public Pose minus(Pose pose) {
            return this.plus(pose.times(-1));
        }

        public Pose2d toPose2d() {
            return new Pose2d(this.strafe, this.forward, this.heading);
        }
    }
}
