package org.firstinspires.ftc.teamcode.commands.subsystems;

import android.util.Pair;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import java.util.stream.Stream;

@Config
public class OdometrySubsystem {

    public static double PARALLEL_UP = 0, PARALLEL_DOWN = 1;
    public static double PERPENDICULAR_UP = 0, PERPENDICULAR_DOWN = 1;

    public OdometrySubsystem(final OpMode opMode) {
        this(
                new SimpleServo(opMode.hardwareMap, "odo_left", 0, 180),
                new SimpleServo(opMode.hardwareMap, "odo_right", 0, 180),
                new SimpleServo(opMode.hardwareMap, "odo_back", 0, 1800)
        );

        if (opMode.getClass().isAnnotationPresent(TeleOp.class)) raise();
        else lower();
    }

    private OdometrySubsystem(ServoEx left, ServoEx right, ServoEx back) {
        left.setInverted(true);

        Wheels.LEFT.setServo(left);
        Wheels.RIGHT.setServo(right);
        Wheels.PERPENDICULAR.setServo(back);

        Wheels.LEFT.createSpline(new Pair<>(0.0, 93.0), new Pair<>(1.0, 125.0));
        Wheels.RIGHT.createSpline(new Pair<>(0.0, 87.0), new Pair<>(1.0, 115.0));
        Wheels.PERPENDICULAR.createSpline(new Pair<>(0.0, 400.0), new Pair<>(0.5, 270.0), new Pair<>(1.0, 180.0));
    }

    public void raise() {
        Wheels.LEFT.setToPosition(PARALLEL_UP);
        Wheels.RIGHT.setToPosition(PARALLEL_UP);
        Wheels.PERPENDICULAR.setToPosition(PERPENDICULAR_UP);
    }

    public void lower() {
        Wheels.LEFT.setToPosition(PARALLEL_DOWN);
        Wheels.RIGHT.setToPosition(PARALLEL_DOWN);
        Wheels.PERPENDICULAR.setToPosition(PERPENDICULAR_DOWN);
    }

    private enum Wheels {
        LEFT, RIGHT, PERPENDICULAR;

        private PolynomialSplineFunction spline;
        private ServoEx servo;

        public void setServo(ServoEx servo) {
            this.servo = servo;
        }

        @SafeVarargs
        public final void createSpline(Pair<Double, Double>... points) {
            double[] x = Stream.of(points)
                    .mapToDouble(p -> p.first)
                    .toArray();
            double[] y = Stream.of(points)
                    .mapToDouble(p -> p.second)
                    .toArray();

            spline = new SplineInterpolator().interpolate(x, y);
        }

        public void setToPosition(double position) throws RuntimeException {
            if (servo == null)
                throw new RuntimeException("The servomotor of the" + this + "odometry isn't initialized");

            if (spline == null)
                throw new RuntimeException("The positions of the " + this + "odometry weren't generated.");

            servo.setPosition(spline.value(position));
        }
    }
}
