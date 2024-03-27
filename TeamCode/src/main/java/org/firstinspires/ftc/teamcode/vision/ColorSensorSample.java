package org.firstinspires.ftc.teamcode.vision;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Collectors;

@TeleOp
@Disabled
public class ColorSensorSample extends CommandOpMode {
    public static double GAIN = 1;
    public static boolean ENABLE_COLOR_PREVIEW = true;

    private RevColorSensorV3 colorSensor;
    private List<LynxModule> hubs;
    private View relativeLayout;

    public void initialize() {
        hubs = hardwareMap.getAll(LynxModule.class);
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color_sensor");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(25);

        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
        colorSensor.setGain((float) GAIN);

        if (ENABLE_COLOR_PREVIEW)
            loadPreviewLayout();

        schedule(
                new ParallelCommandGroup(
                        // Miscellaneous
                        new RunCommand(() -> {
                            telemetry.addData("Current Gain", colorSensor.getGain());
                            telemetry.addData("Distance (CM)", "%.3f", colorSensor.getDistance(DistanceUnit.CM));
                        }),
                        // HSV conversion test
                        new RunCommand(() -> {
                            float[] hsv = new float[3];
                            Color.colorToHSV(colorSensor.getNormalizedColors().toColor(), hsv);

                            telemetry.addData("Hue", hsv[0]);
                            telemetry.addData("Saturation", hsv[1]);
                            telemetry.addData("Value", hsv[2]);

                            if (ENABLE_COLOR_PREVIEW && relativeLayout != null)
                                relativeLayout.post(() -> relativeLayout.setBackgroundColor(Color.HSVToColor(hsv)));
                        }),
                        // Comparison between template normalization and custom
                        new RunCommand(() -> {
                            NormalizedRGBA normalizedTemplate = colorSensor.getNormalizedColors();
                            List<Integer> rawColors = Arrays.asList(colorSensor.red(), colorSensor.green(), colorSensor.blue());

                            double maxVal = rawColors.stream()
                                    .filter(val -> val > 255)
                                    .max(Comparator.comparingInt(val -> val)).orElse(255);
                            List<Integer> normalizedCustom = rawColors.stream()
                                    .map(val -> (int) (val / maxVal * 255)).collect(Collectors.toList());

                            telemetry.addData("Template Normalization",
                                    Arrays.asList(
                                            normalizedTemplate.red,
                                            normalizedTemplate.green,
                                            normalizedTemplate.blue
                                    )
                            );
                            telemetry.addData("Custom Normalization", normalizedCustom);
                        }),
                        new RunCommand(telemetry::update)
                )
        );
    }

    @Override
    public void run() {
        super.run();
        hubs.forEach(LynxModule::clearBulkCache);

        if (colorSensor.getGain() != GAIN)
            colorSensor.setGain((float) GAIN);

        if (ENABLE_COLOR_PREVIEW && relativeLayout == null)
            loadPreviewLayout();
    }

    private void loadPreviewLayout() {
        int viewId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(viewId);
    }
}
