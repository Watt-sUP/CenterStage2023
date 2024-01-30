package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.localization.Localizer;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

abstract class GyroTrackingWheelLocalizer implements Localizer {

    private final DecompositionSolver forwardSolver;
    private Pose2d poseEstimate = new Pose2d();
    private Pose2d poseVelocity = null;
    private List<Double> lastWheelPositions = new ArrayList<>();

    public GyroTrackingWheelLocalizer(List<Pose2d> wheelPoses) {
        assert wheelPoses.size() == 3 : "3 wheel positions must be provided";

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
        if (!wheelPositions.isEmpty()) {
            List<Double> wheelDeltas = IntStream.range(0, wheelPositions.size())
                    .mapToObj(i -> wheelPositions.get(i) - lastWheelPositions.get(i))
                    .collect(Collectors.toList());
            Pose2d robotPoseDelta = calculatePoseDelta(wheelDeltas);
            poseEstimate = Kinematics.relativeOdometryUpdate(poseEstimate, robotPoseDelta);
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