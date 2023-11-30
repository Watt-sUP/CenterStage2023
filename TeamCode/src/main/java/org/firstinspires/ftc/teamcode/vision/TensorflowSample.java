package org.firstinspires.ftc.teamcode.vision;

import android.os.Environment;
import android.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@TeleOp(name = "Vision: TensorFlow Sample", group = "Vision")
public class TensorflowSample extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        TfodProcessor tensorflowProcessor = new TfodProcessor.Builder()
                .setIsModelTensorFlow2(true)
                .setModelAspectRatio(4.0 / 3.0)
                .setModelInputSize(640)
                .setModelFileName(Environment.getExternalStorageDirectory().getPath() + "/FIRST/tflitemodels/red_prop.tflite")
                .setModelLabels(new String[]{"Red Prop"})
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(tensorflowProcessor)
                .setAutoStopLiveView(true)
                .build();

        visionPortal.stopLiveView();
        tensorflowProcessor.setMinResultConfidence(0.6F);
        while (opModeInInit()) {
            if (isStopRequested())
                return;

            List<Recognition> detections = tensorflowProcessor.getRecognitions();
            telemetry.addData("# Objects Detected", detections.size());

            float best_conf = 0;
            Recognition best_detection = null;
            for (Recognition detection : detections) {
                if (detection.getConfidence() > best_conf) {
                    best_conf = 0;
                    best_detection = detection;
                }
            }

            if (best_detection != null) {
                telemetry.addData("Confidence", best_detection.getConfidence());
                telemetry.addData("X, Y", new Pair<>(
                        (best_detection.getLeft() + best_detection.getRight()) / 2,
                        (best_detection.getTop() + best_detection.getBottom()) / 2
                ).toString());
                telemetry.addData("Size", new Pair<>(
                        best_detection.getWidth(),
                        best_detection.getHeight()
                ).toString());
            }

            telemetry.update();
        }

        tensorflowProcessor.shutdown();
        visionPortal.close();
        while (opModeIsActive()) {
            if (isStopRequested())
                return;
        }
    }
}
