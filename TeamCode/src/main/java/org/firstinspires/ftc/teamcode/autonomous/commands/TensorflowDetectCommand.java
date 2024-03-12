package org.firstinspires.ftc.teamcode.autonomous.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.autonomous.assets.AllianceLocation;
import org.firstinspires.ftc.teamcode.autonomous.assets.PropLocations;
import org.firstinspires.ftc.teamcode.subsystems.TensorflowSubsystem;

import java.util.function.BooleanSupplier;

public class TensorflowDetectCommand extends CommandBase {

    private final TensorflowSubsystem detector;
    private final AllianceLocation robotLocation;
    public PropLocations detectedLocation = PropLocations.LEFT;
    private BooleanSupplier endCondition = () -> false;

    public TensorflowDetectCommand(TensorflowSubsystem tensorflow, AllianceLocation location) {
        detector = tensorflow;
        robotLocation = location;

        addRequirements(detector);
    }

    public TensorflowDetectCommand(HardwareMap hardwareMap, AllianceLocation location) {
        robotLocation = location;

        if (location.color == 1)
            detector = new TensorflowSubsystem(hardwareMap, "Webcam 1",
                    "red_prop.tflite", "Red Prop");
        else detector = new TensorflowSubsystem(hardwareMap, "Webcam 1",
                "blue_prop.tflite", "Blue Prop");

        addRequirements(detector);
    }

    public void setEndCondition(BooleanSupplier endCondition) {
        this.endCondition = endCondition;
    }

    @Override
    public void initialize() {
        detector.setMinConfidence(0.8);
        detectedLocation = robotLocation.getHiddenCase();
    }

    @Override
    public void execute() {
        Recognition bestDetection = detector.getBestDetection();
        detectedLocation = robotLocation.getHiddenCase();
        TelemetryPacket packet = new TelemetryPacket();

        if (bestDetection != null) {
            double x = (bestDetection.getLeft() + bestDetection.getRight()) / 2.0;
            if (x < bestDetection.getImageWidth() / 2.0)
                detectedLocation = robotLocation.getVisibleCases().first;
            else detectedLocation = robotLocation.getVisibleCases().second;

            packet.put("Detection X", x);
            packet.put("Detection case", detectedLocation);
        } else packet.addLine("Team Prop not found");
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    @Override
    public boolean isFinished() {
        return endCondition.getAsBoolean();
    }

    @Override
    public void end(boolean interrupted) {
        if (detector.isActive())
            detector.shutdown();
    }
}
