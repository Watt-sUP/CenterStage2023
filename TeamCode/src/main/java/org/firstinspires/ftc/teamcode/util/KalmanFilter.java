package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.util.Angle;

public class KalmanFilter {
    public double Q = .1, R = .4;
    public double p = 1, K = .5;
    private boolean isAngle = false;    private double x = 0, last_x = x, last_p = p;
    public KalmanFilter(double processCovariance, double sensorCovariance) {
        setCovariances(processCovariance, sensorCovariance);
    }

    public KalmanFilter(double initialState) {
        setState(initialState);
    }

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

    public double update(double processChange, double sensorMeasurement) {
        x = last_x + processChange;
        p = last_p + Q;

        K = p / (p + R);

        if (!isAngle)
            x = x + K * (sensorMeasurement - x);
        else
            x = x + K * (normalizeAngleDifference(Angle.norm(sensorMeasurement), Angle.norm(x)));

        p = (1 - K) * p;

        last_x = x;
        last_p = p;

        return isAngle ? Angle.norm(x) : x;
    }

    private double normalizeAngleDifference(double angle1, double angle2) {
        double difference = Math.abs(angle2 - angle1);
        return Angle.normDelta((difference + Math.PI) % (2 * Math.PI) - Math.PI);
    }


}
