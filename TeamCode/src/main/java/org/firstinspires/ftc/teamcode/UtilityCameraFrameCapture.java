/*
 * Copyright (c) 2023 FIRST
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.util.Pair;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.commands.subsystems.CollectorSubsystem;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Locale;
import java.util.Random;
import java.util.concurrent.TimeUnit;

/*
 * This OpMode helps calibrate a webcam or RC phone camera, useful for AprilTag pose estimation
 * with the FTC VisionPortal.   It captures a camera frame (image) and stores it on the Robot Controller
 * (Control Hub or RC phone), with each press of the gamepad button X (or Square).
 * Full calibration instructions are here:
 *
 *  https://ftc-docs.firstinspires.org/camera-calibration
 *
 * In Android Studio, copy this class into your "teamcode" folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 *
 * In OnBot Java, use "Add File" to add this OpMode from the list of Samples.
 */

@Config
@TeleOp(name = "Utility: Camera Frame Capture", group = "Utility")
//@Disabled
public class UtilityCameraFrameCapture extends LinearOpMode {
    /*
     * EDIT THESE PARAMETERS AS NEEDED
     */

    public static boolean RANDOMIZE_LIGHTING = false;
    public static String IMAGE_NAME = "CameraCalibrationFrame";

    // Internal state
    boolean lastX;
    int frameCount;
    long capReqTime;

    @Override
    public void runOpMode() {
        VisionPortal portal;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        CollectorSubsystem collectorSystem = new CollectorSubsystem(
                new SimpleServo(hardwareMap, "v4b_left", 0, 180),
                new SimpleServo(hardwareMap, "v4b_right", 0, 180),
                new SimpleServo(hardwareMap, "claw", 0, 300)
        );

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();

        while (!isStopRequested()) {
            boolean x = gamepad1.square;

            drive.setWeightedDrivePower(new Pose2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            ));

            if (x && !lastX) {
                portal.saveNextFrameRaw(String.format(Locale.US, IMAGE_NAME + "-%06d", frameCount++));
                capReqTime = System.currentTimeMillis();

                if (RANDOMIZE_LIGHTING)
                    randomizeLighting(portal);
            }

            lastX = x;

            telemetry.addLine("######## Camera Capture Utility ########");
            telemetry.addLine(" > Press Square to capture a frame");
            telemetry.addData(" > Camera Status", portal.getCameraState());
            telemetry.addData("Photos Taken", frameCount);

            if (capReqTime != 0) {
                telemetry.addLine("\nCaptured Frame!");
            }

            if (capReqTime != 0 && System.currentTimeMillis() - capReqTime > 1000) {
                capReqTime = 0;
            }

            telemetry.update();
        }
    }

    public void randomizeLighting(VisionPortal portal) {
        ExposureControl exposure = portal.getCameraControl(ExposureControl.class);
        GainControl gain = portal.getCameraControl(GainControl.class);

        if (exposure.getMode() != ExposureControl.Mode.Manual)
            exposure.setMode(ExposureControl.Mode.Manual);

        Pair<Integer, Integer> gain_limits = new Pair<>(gain.getMinGain(), gain.getMaxGain());
        Pair<Long, Long> exposure_limits = new Pair<>(exposure.getMinExposure(TimeUnit.MILLISECONDS) + 1, exposure.getMaxExposure(TimeUnit.MILLISECONDS));

        Random random = new Random();
        long new_exposure = random.nextInt((int) (exposure_limits.second - exposure_limits.first)) + exposure_limits.first;
        int new_gain = random.nextInt(gain_limits.second - gain_limits.first) + gain_limits.first;

        exposure.setExposure(new_exposure, TimeUnit.MILLISECONDS);
        gain.setGain(new_gain);
    }
}
