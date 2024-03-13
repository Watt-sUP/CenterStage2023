package org.firstinspires.ftc.teamcode.commands.subsystems;

import android.os.Environment;

import androidx.annotation.Nullable;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.autonomous.assets.AllianceLocation;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.Comparator;
import java.util.List;

public class TensorflowSubsystem extends SubsystemBase {

    private final TfodProcessor tensorflowProcessor;
    public VisionPortal portal;

    public TensorflowSubsystem(HardwareMap hardwareMap, String cameraName, String modelName, String... labels) {
        tensorflowProcessor = new TfodProcessor.Builder()
                .setIsModelTensorFlow2(true)
                .setModelAspectRatio(4.0 / 3.0)
                .setModelInputSize(640)
                .setModelFileName(Environment.getExternalStorageDirectory().getPath() + "/FIRST/tflitemodels/" + modelName)
                .setModelLabels(labels)
                .build();

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, cameraName))
                .addProcessor(tensorflowProcessor)
                .setAutoStopLiveView(true)
                .build();
    }

    public TensorflowSubsystem(HardwareMap hardwareMap, AllianceLocation location) {
        this(hardwareMap, "Webcam 1",
                (location.color == 1 ? "red" : "blue") + "_prop.tflite",
                (location.color == 1 ? "Red" : "Blue") + "Prop");
    }

    public void shutdown() {
        tensorflowProcessor.shutdown();
        portal.close();
    }

    public void setMinConfidence(double confidence) {
        tensorflowProcessor.setMinResultConfidence((float) confidence);
    }

    @Nullable
    public Recognition getBestDetection() {
        List<Recognition> detections = tensorflowProcessor.getRecognitions();
        return detections.stream()
                // Filter out detections bigger than half of the image (likely junk)
                .filter(detection -> detection.getWidth() < (detection.getImageWidth() / 2.0))
                .max(Comparator.comparingDouble(Recognition::getConfidence))
                .orElse(null);
    }
}