package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.hardware.IMU;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.KalmanFilter;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

abstract class GyroTrackingWheelLocalizer implements Localizer {

    private final DecompositionSolver forwardSolver;
    private final KalmanFilter filter = new KalmanFilter();
    private List<Double> lastWheelPositions = new ArrayList<>();
    private final IMU gyroscope;
    private final FtcDashboard dashboard;
    private Pose2d poseEstimate = new Pose2d(), poseVelocity = null;
    private double gyroOffset = 0;

    public GyroTrackingWheelLocalizer(@NonNull List<Pose2d> wheelPoses, @Nullable IMU imu) {
        assert wheelPoses.size() == 3 : "3 wheel positions must be provided";
        dashboard = FtcDashboard.getInstance();
        gyroscope = imu;

        if (gyroscope != null) {
            gyroscope.resetYaw();
            filter.setIsAngle(true);
            filter.setCovariances(0.1, 0.4);
        }

        Array2DRowRealMatrix inverseMatrix = new Array2DRowRealMatrix(3, 3);
        for (int i = 0; i <= 2; i++) {
            Vector2d orientationVector = wheelPoses.get(i).headingVec();
            Vector2d positionVector = wheelPoses.get(i).vec();

            inverseMatrix.setEntry(i, 0, orientationVector.getX());
            inverseMatrix.setEntry(i, 1, orientationVector.getY());
            inverseMatrix.setEntry(i, 2,
                    positionVector.getX() * orientationVector.getY() - positionVector.getY() * orientationVector.getX());
        }

        forwardSolver = new LUDecomposition(inverseMatrix).getSolver();
        assert forwardSolver.isNonSingular() : "The specified configuration cannot support full localization";
    }

    private Pose2d calculatePoseDelta(List<Double> wheelDeltas) {
        int size = wheelDeltas.size();
        double[] deltasArray = new double[size];

        for (int i = 0; i < size; i++)
            deltasArray[i] = wheelDeltas.get(i);

        RealMatrix rawPoseDelta = forwardSolver.solve(
                MatrixUtils.createRealMatrix(new double[][]{deltasArray})
                        .transpose()
        );

        return new Pose2d(
                rawPoseDelta.getEntry(0, 0),
                rawPoseDelta.getEntry(1, 0),
                rawPoseDelta.getEntry(2, 0)
        );
    }

    @Override
    public void update() {
        List<Double> wheelPositions = getWheelPositions();
        if (!lastWheelPositions.isEmpty()) {
            List<Double> wheelDeltas = IntStream.range(0, wheelPositions.size())
                    .mapToObj(i -> wheelPositions.get(i) - lastWheelPositions.get(i))
                    .collect(Collectors.toList());

            Pose2d robotPoseDelta = calculatePoseDelta(wheelDeltas);

            if (gyroscope == null)
                poseEstimate = Kinematics.relativeOdometryUpdate(poseEstimate, robotPoseDelta);
            else {
                Pose2d odometryEstimate = Kinematics.relativeOdometryUpdate(poseEstimate, robotPoseDelta);
                double gyroAngle = gyroOffset + gyroscope.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                double odometryAngle = odometryEstimate.getHeading() - poseEstimate.getHeading();

                double filterAngle = filter.update(odometryAngle, gyroAngle);

                TelemetryPacket packet = new TelemetryPacket();
                packet.put("heading (odometry)", Math.toDegrees(odometryEstimate.getHeading()));
                packet.put("heading (gyro)", Math.toDegrees(gyroAngle));
                packet.put("heading (kalman)", Math.toDegrees(filterAngle));
                dashboard.sendTelemetryPacket(packet);

                poseEstimate = new Pose2d(odometryEstimate.vec(), filterAngle);
            }
        }

        List<Double> wheelVelocities = getWheelVelocities();
        if (wheelVelocities != null)
            poseVelocity = calculatePoseDelta(wheelVelocities);

        lastWheelPositions = wheelPositions;
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d newPose) {
        lastWheelPositions = new ArrayList<>();
        poseEstimate = newPose;

        if (gyroscope != null) {
            gyroscope.resetYaw();
            gyroOffset = newPose.getHeading();
            filter.setState(newPose.getHeading());
        }
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return poseVelocity;
    }

    abstract List<Double> getWheelPositions();

    @Nullable
    public List<Double> getWheelVelocities() {
        return null;
    }
}
