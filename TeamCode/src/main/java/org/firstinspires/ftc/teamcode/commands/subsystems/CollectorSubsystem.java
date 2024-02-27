package org.firstinspires.ftc.teamcode.commands.subsystems;

import android.util.Pair;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.analysis.interpolation.LinearInterpolator;
import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import java.util.concurrent.TimeUnit;
import java.util.stream.Stream;

@Config
public class CollectorSubsystem extends SubsystemBase {
    public static Double LOWER_LIFT = 0.74, RAISE_LIFT = 0.075, STACK_LIFT = 0.665;
    private final ServoEx claw;
    private final LiftServo left, right;
    /*
     Timer responsible for periodic check:
     - Starts/resets once the claw is closed (something is collected)
     - Runs until it's done
     - Claw gets raised
     - Timer is paused until the cycle repeats
    */
    private final Timing.Timer clampTimer = new Timing.Timer(275, TimeUnit.MILLISECONDS);

    public enum ClampState {
        CLOSED,
        OPENED
    }

    public ClampState clamping = ClampState.CLOSED;
    public LiftState location = LiftState.STACK;

    public CollectorSubsystem(final HardwareMap hardwareMap) {
        this(
                new SimpleServo(hardwareMap, "v4b_left", 0, 220),
                new SimpleServo(hardwareMap, "v4b_right", 0, 220),
                new SimpleServo(hardwareMap, "claw", 0, 220)
        );
    }

    private CollectorSubsystem(ServoEx liftL, ServoEx liftR, ServoEx clamp) {
        left = new LiftServo(liftL);
        right = new LiftServo(liftR);

        left.setInverted(false);
        right.setInverted(true);

        left.createSpline(new Pair<>(0.0, 15.0), new Pair<>(0.5, 112.5), new Pair<>(1.0, 210.0));
        right.createSpline(new Pair<>(0.0, 6.5), new Pair<>(0.5, 101.0), new Pair<>(1.0, 199.0));

        claw = clamp;
        claw.setInverted(true);
        claw.turnToAngle(90);
        this.setLiftLocation(LiftState.RAISED);
    }

    public void setLiftLocation(LiftState target) {
        if (target == location)
            return;

        // Claw needs to be closed to avoid hitting the belts
        if (clamping == ClampState.OPENED && (target == LiftState.RAISED || location == LiftState.RAISED))
            this.toggleClamp();

        switch (target) {
            case RAISED:
                left.setToPosition(RAISE_LIFT);
                right.setToPosition(RAISE_LIFT);
                break;

            case STACK:
                left.setToPosition(STACK_LIFT);
                right.setToPosition(STACK_LIFT);
                break;

            case LOWERED:
                left.setToPosition(LOWER_LIFT);
                right.setToPosition(LOWER_LIFT);
                break;
        }
        location = target;
    }

    public void toggleLiftLocation() {
        switch (location) {
            case LOWERED:
                this.setLiftLocation(LiftState.RAISED);
                break;
            case RAISED:
                this.setLiftLocation(LiftState.STACK);
                break;
            case STACK:
                // Open the claw when it's fully lowered
                if (clamping == ClampState.CLOSED)
                    this.toggleClamp();

                this.setLiftLocation(LiftState.LOWERED);
                break;
        }
    }

    public void adjustLiftPosition(Double adjustment) {
        double new_pos = left.getCurrentPosition() + adjustment;

        left.setToPosition(new_pos);
        right.setToPosition(new_pos);
    }

    public void toggleClamp() {
        switch (clamping) {
            case OPENED:
                // The code of the timer resets it every time .start() is called (equivalent to a .reset())
                if (location != LiftState.RAISED) // Can't collect when raised
                    clampTimer.start();

                claw.turnToAngle(90);
                clamping = ClampState.CLOSED;
                break;
            case CLOSED:
                // Don't open the claw fully when the lift is raised to avoid the belts
                claw.turnToAngle(location != LiftState.RAISED ? 50 : 70);
                clamping = ClampState.OPENED;
                break;
        }
    }

    public void setClampPosition(double angle) {
        claw.turnToAngle(angle);
        clamping = (angle <= 70) ? ClampState.OPENED : ClampState.CLOSED;
    }

    // Periodic check: Automatically raise the claw 275ms after collection
    @Override
    public void periodic() {
        // Timer keeps going even after it's done, so pause it to avoid repetition
        if (clampTimer.done() && clampTimer.isTimerOn()) {
            this.setLiftLocation(LiftState.RAISED);
            clampTimer.pause();
        }
    }

    public enum LiftState {
        LOWERED,
        RAISED,
        STACK
    }

    private static class LiftServo {
        private final ServoEx servo;
        private Double currentPosition = 0.0;
        private PolynomialSplineFunction spline;

        LiftServo(ServoEx servo) {
            this.servo = servo;
        }

        /**
         * <p>Interpolates the servo's position to a relevant angle range using a cubic spline.</p>
         * <p>Typical usage consists of restricting the servomotor to certain angles with a range from 0 to 1.</p>
         *
         * @param positions A list of sample points for the generator to use (must use at least 2 pairs)
         */
        @SafeVarargs
        public final void createSpline(Pair<Double, Double>... positions) {
            double[] x = Stream.of(positions)
                    .mapToDouble(p -> p.first)
                    .toArray();
            double[] y = Stream.of(positions)
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
         * @throws RuntimeException         If the positions weren't generated
         * @throws IllegalArgumentException If the desired position is out of the generated range
         */
        public void setToPosition(double position) throws RuntimeException {
            if (spline == null)
                throw new RuntimeException("The positions of the servomotor weren't generated.");

            if (!spline.isValidPoint(position)) {
                double[] knots = spline.getKnots();
                throw new IllegalArgumentException(String.format("Unable to access position %2f. " +
                        "Spline positions range from [%.2f, %.2f]", position, knots[0], knots[knots.length - 1]));
            }
            servo.turnToAngle(spline.value(position));
            currentPosition = position;
        }

        public void setInverted(boolean isInverted) {
            servo.setInverted(isInverted);
        }

        public Double getCurrentPosition() {
            return currentPosition;
        }
    }
}
