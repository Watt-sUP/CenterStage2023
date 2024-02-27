package org.firstinspires.ftc.teamcode.commands.subsystems;

import android.util.Pair;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.apache.commons.math3.analysis.interpolation.LinearInterpolator;
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

        Wheels.LEFT.createSpline(new Pair<>(0.0, 110.0), new Pair<>(1.0, 128.0));
        Wheels.RIGHT.createSpline(new Pair<>(0.0, 99.0), new Pair<>(1.0, 119.0));

        // TODO: Find positions for this wheel
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

        /**
         * <p>Creates an interpolator for position generation in a relevant range.</p>
         * <p>Typical usage consists of restricting the servomotor to certain angles with a range from 0 to 1.</p>
         *
         * @param points A list of sample points for the generator to use (must use at least 2 pairs)
         */
        @SafeVarargs
        public final void createSpline(Pair<Double, Double>... points) {
            double[] x = Stream.of(points)
                    .mapToDouble(p -> p.first)
                    .toArray();
            double[] y = Stream.of(points)
                    .mapToDouble(p -> p.second)
                    .toArray();

            if (x.length == 2)
                spline = new LinearInterpolator().interpolate(x, y);
            else spline = new SplineInterpolator().interpolate(x, y);
        }

        /**
         * Sets the position of the wheel's servomotor based on its interpolated function.
         *
         * @param position The position to set the wheel to
         * @throws RuntimeException         If the servo wasn't set or positions weren't generated
         * @throws IllegalArgumentException If the desired position is out of the generated range
         */
        public void setToPosition(double position) throws RuntimeException {
            if (servo == null)
                throw new RuntimeException(String.format("The servomotor of the %s odometry isn't initialized", this));

            if (spline == null)
                throw new RuntimeException(String.format("The positions of the %s odometry weren't generated.", this));

            if (!spline.isValidPoint(position)) {
                double[] knots = spline.getKnots();
                throw new IllegalArgumentException(String.format("Unable to access position %2f. " +
                        "Spline positions range from [%.2f, %.2f]", position, knots[0], knots[knots.length - 1]));
            }
            servo.setPosition(spline.value(position));
        }

        @NonNull
        @Override
        public String toString() {
            return this.name().toLowerCase();
        }
    }
}
