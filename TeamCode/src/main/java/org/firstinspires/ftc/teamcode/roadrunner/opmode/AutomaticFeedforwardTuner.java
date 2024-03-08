package org.firstinspires.ftc.teamcode.roadrunner.opmode;

import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.MAX_RPM;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.rpmToVelocity;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.commands.subsystems.CollectorSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.RegressionUtil;

import java.util.ArrayList;
import java.util.List;

/*
 * Op mode for computing kV, kStatic, and kA from various drive routines. For the curious, here's an
 * outline of the procedure:
 *   1. Slowly ramp the motor power and record encoder values along the way.
 *   2. Run a linear regression on the encoder velocity vs. motor power plot to obtain a slope (kV)
 *      and an optional intercept (kStatic).
 *   3. Accelerate the robot (apply constant power) and record the encoder counts.
 *   4. Adjust the encoder data based on the velocity tuning data and find kA with another linear
 *      regression.
 */
@Config
@Autonomous(group = "drive")
public class AutomaticFeedforwardTuner extends LinearOpMode {
    public static double MAX_POWER = 0.7;
    public static double DISTANCE = 100; // in

    @Override
    public void runOpMode() throws InterruptedException {

        OdometrySubsystem odometry = new OdometrySubsystem(this);
        CollectorSubsystem intake = new CollectorSubsystem(hardwareMap);

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        NanoClock clock = NanoClock.system();
        waitForStart();

        if (isStopRequested()) return;

        telemetry.clearAll();
        telemetry.addLine("Running...");
        telemetry.update();

        double maxVel = rpmToVelocity(MAX_RPM);
        double finalVel = MAX_POWER * maxVel;
        double accel = (finalVel * finalVel) / (2.0 * DISTANCE);
        double rampTime = Math.sqrt(2.0 * DISTANCE / accel);

        List<Double> timeSamples = new ArrayList<>();
        List<Double> positionSamples = new ArrayList<>();
        List<Double> powerSamples = new ArrayList<>();

        drive.setPoseEstimate(new Pose2d());

        double startTime = clock.seconds();
        while (!isStopRequested()) {
            double elapsedTime = clock.seconds() - startTime;
            if (elapsedTime > rampTime) {
                break;
            }
            double vel = accel * elapsedTime;
            double power = vel / maxVel;

            timeSamples.add(elapsedTime);
            positionSamples.add(drive.getPoseEstimate().getX());
            powerSamples.add(power);

            drive.setDrivePower(new Pose2d(power, 0.0, 0.0));
            drive.updatePoseEstimate();
        }
        drive.setDrivePower(new Pose2d(0.0, 0.0, 0.0));

        RegressionUtil.RampResult rampResult = RegressionUtil.fitRampData(
                timeSamples, positionSamples, powerSamples, true);

        telemetry.clearAll();
        telemetry.addLine("Quasi-static ramp up test complete");
        telemetry.addLine(Misc.formatInvariant("kV = %.5f, kStatic = %.5f (R^2 = %.2f)",
                rampResult.kV, rampResult.kStatic, rampResult.rSquare));
        telemetry.update();

        while (!isStopRequested()) {
            idle();
        }
    }
}