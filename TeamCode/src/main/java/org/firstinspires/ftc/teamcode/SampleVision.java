package org.firstinspires.ftc.teamcode;

import android.util.Pair;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.TeamPropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

@Config
@TeleOp
public class SampleVision extends LinearOpMode {

    public static TeamPropProcessor.PropColor TARGET_COLOR = TeamPropProcessor.PropColor.RED;
    public static Double MIN_AREA = 100., LEFT = 200., RIGHT = 400.;
    public static boolean SHOW_MASK = false;
    public static Scalar MIN_VALUES = new Scalar(110, 80, 40), MAX_VALUES = new Scalar(125, 255, 255);

    @Override
    public void runOpMode() throws InterruptedException {
        TeamPropProcessor processor = new TeamPropProcessor(TARGET_COLOR, MIN_AREA, LEFT, RIGHT, SHOW_MASK);
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .addProcessor(processor)
                .setAutoStopLiveView(true)
                .build();

        while (opModeInInit()) {
            if (isStopRequested())
                return;

            Pair<Scalar, Scalar> currentThresholds = processor.getThresholds(TARGET_COLOR);
            if (!new Pair<>(MIN_VALUES, MAX_VALUES).equals(currentThresholds))
                processor.setThresholds(TARGET_COLOR, MIN_VALUES, MAX_VALUES);

            telemetry.addData("FPS", visionPortal.getFps());
            telemetry.addData("Contour Area", processor.getLargestContourArea());
            telemetry.addData("Contour Coords", processor.getLargestContourCoords().toString());
            telemetry.addData("Minimum Threshold", MIN_VALUES.toString());
            telemetry.addData("Maximum Threshold", MAX_VALUES.toString());
            telemetry.update();
        }

        waitForStart();
        visionPortal.close();
    }
}
