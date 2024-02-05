package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.util.Angle;

import java.util.Locale;

public class KalmanFilter {
    public double Q = .1, R = .4;
    public double p = 1, K = .5;
    private boolean isAngle = false;
    public double update(double processChange, double sensorMeasurement) {
        x = last_x + processChange;
        p = last_p + Q;

        K = p / (p + R);

        if (!isAngle)
            x = x + K * (sensorMeasurement - x);
        else
            x = x + K * (Angle.normDelta(Angle.norm(sensorMeasurement) - Angle.norm(x)));

        p = (1 - K) * p;

        last_x = x;
        last_p = p;

        return isAngle ? Angle.norm(x) : x;
    }    private double x = 0, last_x = x, last_p = p;



    public KalmanFilter() {
    }

    public void setState(double state) {
        x = state;
        last_x = state;
    }

    public void setIsAngle(boolean isAngle) {
        this.isAngle = isAngle;
    }

    public void setCovariances(double process, double sensor) {
        Q = process;
        R = sensor;
    }

    @NonNull
    @Override
    public String toString() {
        return String.format(Locale.US, "x: %.2f, kGain: %.4g", x, K);
    }
}
