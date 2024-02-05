package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.util.Angle;

import java.util.function.DoubleSupplier;

/**
 * <p>Class for a simple complementary filter.</p>
 * <p>The complementary filter works by setting a high-pass filter on a measurement and a low-pass filter on the other.</p>
 */
public class CompFilter {
    public double alpha = 0.5;
    private boolean isAngle = false;
    private DoubleSupplier a = null, b = null;

    public CompFilter() {
    }

    public CompFilter(@NonNull DoubleSupplier a, @NonNull DoubleSupplier b) {
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
        if (!isAngle)
            return alpha * a + (1 - alpha) * b;
        else {
            double angle_a = Angle.normDelta(a), angle_b = Angle.normDelta(b);
            if (angle_a * angle_b < 0) {
                angle_a = Angle.norm(angle_a);
                angle_b = Angle.norm(angle_b);
            }

            return Angle.norm(alpha * angle_a + (1 - alpha) * angle_b);
        }
    }

    public void setIsAngle(boolean isAngle) {
        this.isAngle = isAngle;
    }

    public void setAlpha(double alpha) {
        this.alpha = alpha;
    }
}
