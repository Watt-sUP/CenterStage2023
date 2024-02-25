package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.util.Angle;

import java.util.function.DoubleSupplier;

/**
 * <p>Class for a simple complementary filter.</p>
 * <p>The complementary filter works by setting a high-pass filter on a measurement and a low-pass filter on the other.</p>
 */
public class CompFilter {
    public double alpha = 0.5;
    private double x, last_x;
    private boolean isAngle = false;
    private DoubleSupplier a, b;

    public CompFilter() {
    }

    public CompFilter(DoubleSupplier a, DoubleSupplier b) {
        this.a = a;
        this.b = b;
    }

    @Nullable
    public Double update() {
        if (a == null || b == null)
            return null;

        return update(a.getAsDouble(), b.getAsDouble());
    }

    public double update(double a, double b) {
        if (!isAngle) {
            last_x = x;
            x = alpha * a + (1 - alpha) * b;
        } else {
            double angle_a = Angle.norm(a), angle_b = Angle.norm(b);

            last_x = x;
            while (Math.abs(angle_a - angle_b) > Math.PI) {
                if (angle_a < angle_b)
                    angle_a += (2 * Math.PI);
                else angle_b += (2 * Math.PI);
            }
            x = Angle.norm(alpha * angle_a + (1 - alpha) * angle_b);
        }

        return x;
    }

    public void setIsAngle(boolean isAngle) {
        this.isAngle = isAngle;
    }

    public void setAlpha(double alpha) {
        this.alpha = alpha;
    }

    public double getDelta() {
        return isAngle ? Angle.normDelta(x - last_x) : (x - last_x);
    }

    public void setEstimate(double x) {
        this.x = x;
        last_x = x;
    }
}
