package org.firstinspires.ftc.teamcode.trajectorysequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.PathContinuityViolationException;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.TemporalMarker;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.util.Angle;

import org.firstinspires.ftc.teamcode.autonomous.assets.AllianceLocation;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.SequenceSegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TrajectorySegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TurnSegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.WaitSegment;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Collectors;

public class TrajectorySequenceBuilder {
    private final double resolution = 0.25;

    private final TrajectoryVelocityConstraint baseVelConstraint;
    private final TrajectoryAccelerationConstraint baseAccelConstraint;

    private TrajectoryVelocityConstraint currentVelConstraint;
    private TrajectoryAccelerationConstraint currentAccelConstraint;

    private final double baseTurnConstraintMaxAngVel, baseTurnConstraintMaxAngAccel;
    private final List<SequenceSegment> sequenceSegments = new ArrayList<>();
    private final List<TemporalMarker> temporalMarkers = new ArrayList<>();
    private final AllianceLocation startLocation;

    private Pose2d lastPose;
    private double currentTurnConstraintMaxAngVel, currentTurnConstraintMaxAngAccel;
    private double absoluteTangent, tangentOffset;

    private boolean setAbsoluteTangent = false;

    private TrajectoryBuilder currentTrajectoryBuilder;

    private double currentDuration;
    private double lastDurationTraj;

    public TrajectorySequenceBuilder(
            Pose2d startPose, AllianceLocation startLocation,
            TrajectoryVelocityConstraint baseVelConstraint,
            TrajectoryAccelerationConstraint baseAccelConstraint,
            double baseTurnConstraintMaxAngVel,
            double baseTurnConstraintMaxAngAccel
    ) {
        this.startLocation = startLocation;
        this.baseVelConstraint = baseVelConstraint;
        this.baseAccelConstraint = baseAccelConstraint;

        this.currentVelConstraint = baseVelConstraint;
        this.currentAccelConstraint = baseAccelConstraint;

        this.baseTurnConstraintMaxAngVel = baseTurnConstraintMaxAngVel;
        this.baseTurnConstraintMaxAngAccel = baseTurnConstraintMaxAngAccel;

        this.currentTurnConstraintMaxAngVel = baseTurnConstraintMaxAngVel;
        this.currentTurnConstraintMaxAngAccel = baseTurnConstraintMaxAngAccel;

        lastPose = convertPose(startPose);
    }

    public TrajectorySequenceBuilder(
            Pose2d startPose,
            TrajectoryVelocityConstraint baseVelConstraint,
            TrajectoryAccelerationConstraint baseAccelConstraint,
            double baseTurnConstraintMaxAngVel,
            double baseTurnConstraintMaxAngAccel
    ) {
        this(
                startPose, null,
                baseVelConstraint, baseAccelConstraint,
                baseTurnConstraintMaxAngVel, baseTurnConstraintMaxAngAccel
        );
    }

    public TrajectorySequenceBuilder lineTo(Vector2d endPosition) {
        return addPath(() -> currentTrajectoryBuilder.lineTo(convertVector(endPosition), currentVelConstraint, currentAccelConstraint));
    }

    public TrajectorySequenceBuilder lineToLinearHeading(Pose2d endPose) {
        return addPath(() -> currentTrajectoryBuilder.lineToLinearHeading(convertPose(endPose), currentVelConstraint, currentAccelConstraint));
    }

    public TrajectorySequenceBuilder lineToSplineHeading(Pose2d endPose) {
        return addPath(() -> currentTrajectoryBuilder.lineToSplineHeading(convertPose(endPose), currentVelConstraint, currentAccelConstraint));
    }

    public TrajectorySequenceBuilder forward(double distance) {
        return addPath(() -> currentTrajectoryBuilder.forward(distance, currentVelConstraint, currentAccelConstraint));
    }

    public TrajectorySequenceBuilder splineTo(Vector2d endPosition, double endHeading) {
        return addPath(() -> {
            Pose2d endPose = convertPose(new Pose2d(endPosition, endHeading));
            currentTrajectoryBuilder.splineTo(endPose.vec(), endPose.getHeading(), currentVelConstraint, currentAccelConstraint);
        });
    }

    // TODO: Remove this function's usages
    @Deprecated
    public TrajectorySequenceBuilder splineTo(Pose2d endPosition) {
        return addPath(() -> currentTrajectoryBuilder.splineTo(endPosition.vec(), endPosition.getHeading(), currentVelConstraint, currentAccelConstraint));
    }

    public TrajectorySequenceBuilder splineToLinearHeading(Pose2d endPose, double endHeading) {
        return addPath(() -> {
            double multiplier = 1;
            if (startLocation != null)
                multiplier = startLocation.color;

            currentTrajectoryBuilder.splineToLinearHeading(convertPose(endPose), endHeading * multiplier, currentVelConstraint, currentAccelConstraint);
        });
    }

    public TrajectorySequenceBuilder splineToSplineHeading(Pose2d endPose, double endHeading) {
        return addPath(() -> {
            double multiplier = 1;
            if (startLocation != null)
                multiplier = startLocation.color;

            currentTrajectoryBuilder.splineToSplineHeading(convertPose(endPose), endHeading * multiplier, currentVelConstraint, currentAccelConstraint);
        });
    }

    private TrajectorySequenceBuilder addPath(AddPathCallback callback) {
        if (currentTrajectoryBuilder == null) newPath();

        try {
            callback.run();
        } catch (PathContinuityViolationException e) {
            newPath();
            callback.run();
        }

        Trajectory builtTraj = currentTrajectoryBuilder.build();

        double durationDifference = builtTraj.duration() - lastDurationTraj;

        lastPose = builtTraj.end();
        currentDuration += durationDifference;

        lastDurationTraj = builtTraj.duration();

        return this;
    }

    public TrajectorySequenceBuilder setTangent(double tangent) {
        setAbsoluteTangent = true;
        absoluteTangent = tangent;

        pushPath();

        return this;
    }

    private TrajectorySequenceBuilder setTangentOffset(double offset) {
        setAbsoluteTangent = false;

        this.tangentOffset = offset;
        this.pushPath();

        return this;
    }

    public TrajectorySequenceBuilder setReversed(boolean reversed) {
        return reversed ? setTangentOffset(Math.toRadians(180.0)) : setTangentOffset(0.0);
    }

    public TrajectorySequenceBuilder setConstraints(
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        this.currentVelConstraint = velConstraint;
        this.currentAccelConstraint = accelConstraint;

        return this;
    }

    public TrajectorySequenceBuilder resetConstraints() {
        this.currentVelConstraint = this.baseVelConstraint;
        this.currentAccelConstraint = this.baseAccelConstraint;

        return this;
    }

    public TrajectorySequenceBuilder setVelConstraint(TrajectoryVelocityConstraint velConstraint) {
        this.currentVelConstraint = velConstraint;

        return this;
    }

    public TrajectorySequenceBuilder resetVelConstraint() {
        this.currentVelConstraint = this.baseVelConstraint;

        return this;
    }

    public TrajectorySequenceBuilder setAccelConstraint(TrajectoryAccelerationConstraint accelConstraint) {
        this.currentAccelConstraint = accelConstraint;

        return this;
    }

    public TrajectorySequenceBuilder resetAccelConstraint() {
        this.currentAccelConstraint = this.baseAccelConstraint;

        return this;
    }

    public TrajectorySequenceBuilder setTurnConstraint(double maxAngVel, double maxAngAccel) {
        this.currentTurnConstraintMaxAngVel = maxAngVel;
        this.currentTurnConstraintMaxAngAccel = maxAngAccel;

        return this;
    }

    public TrajectorySequenceBuilder resetTurnConstraint() {
        this.currentTurnConstraintMaxAngVel = baseTurnConstraintMaxAngVel;
        this.currentTurnConstraintMaxAngAccel = baseTurnConstraintMaxAngAccel;

        return this;
    }

    public TrajectorySequenceBuilder addTemporalMarker(MarkerCallback callback) {
        this.temporalMarkers.add(new TemporalMarker(time -> currentDuration, callback));
        return this;
    }

    public TrajectorySequenceBuilder turn(double angle) {
        return turn(angle, currentTurnConstraintMaxAngVel, currentTurnConstraintMaxAngAccel);
    }

    public TrajectorySequenceBuilder turn(double angle, double maxAngVel, double maxAngAccel) {
        pushPath();

        MotionProfile turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(lastPose.getHeading(), 0.0, 0.0, 0.0),
                new MotionState(lastPose.getHeading() + angle, 0.0, 0.0, 0.0),
                maxAngVel,
                maxAngAccel
        );

        sequenceSegments.add(new TurnSegment(lastPose, angle, turnProfile, Collections.emptyList()));

        lastPose = new Pose2d(
                lastPose.getX(), lastPose.getY(),
                Angle.norm(lastPose.getHeading() + angle)
        );

        currentDuration += turnProfile.duration();

        return this;
    }

    public TrajectorySequenceBuilder waitSeconds(double seconds) {
        pushPath();
        sequenceSegments.add(new WaitSegment(lastPose, seconds, Collections.emptyList()));

        currentDuration += seconds;
        return this;
    }

    public TrajectorySequenceBuilder addTrajectory(Trajectory trajectory) {
        pushPath();

        sequenceSegments.add(new TrajectorySegment(trajectory));
        return this;
    }

    private void pushPath() {
        if (currentTrajectoryBuilder != null) {
            Trajectory builtTraj = currentTrajectoryBuilder.build();
            sequenceSegments.add(new TrajectorySegment(builtTraj));
        }

        currentTrajectoryBuilder = null;
    }

    private void newPath() {
        if (currentTrajectoryBuilder != null)
            pushPath();

        lastDurationTraj = 0.0;

        double tangent = setAbsoluteTangent ? absoluteTangent : Angle.norm(lastPose.getHeading() + tangentOffset);

        currentTrajectoryBuilder = new TrajectoryBuilder(lastPose, tangent, currentVelConstraint, currentAccelConstraint, resolution);
    }

    public TrajectorySequence build() {
        pushPath();

        List<TrajectoryMarker> globalMarkers = convertMarkersToGlobal(temporalMarkers);
        return new TrajectorySequence(projectGlobalMarkersToLocalSegments(globalMarkers, sequenceSegments));
    }

    private List<TrajectoryMarker> convertMarkersToGlobal(List<TemporalMarker> temporalMarkers) {
        // Convert temporal markers
        return temporalMarkers.stream()
                .map(marker ->
                        new TrajectoryMarker(marker.getProducer().produce(currentDuration), marker.getCallback()))
                .collect(Collectors.toList());
    }

    private List<SequenceSegment> projectGlobalMarkersToLocalSegments(List<TrajectoryMarker> markers, List<SequenceSegment> sequenceSegments) {
        if (sequenceSegments.isEmpty()) return Collections.emptyList();

        markers.sort(Comparator.comparingDouble(TrajectoryMarker::getTime));

        int segmentIndex = 0;
        double currentTime = 0;

        for (TrajectoryMarker marker : markers) {
            SequenceSegment segment = null;

            double markerTime = marker.getTime();
            double segmentOffsetTime = 0;

            while (segmentIndex < sequenceSegments.size()) {
                SequenceSegment seg = sequenceSegments.get(segmentIndex);

                if (currentTime + seg.getDuration() >= markerTime) {
                    segment = seg;
                    segmentOffsetTime = markerTime - currentTime;
                    break;
                } else {
                    currentTime += seg.getDuration();
                    segmentIndex++;
                }
            }
            if (segmentIndex >= sequenceSegments.size()) {
                segment = sequenceSegments.get(sequenceSegments.size() - 1);
                segmentOffsetTime = segment.getDuration();
            }

            SequenceSegment newSegment = null;

            if (segment instanceof WaitSegment) {
                WaitSegment thisSegment = (WaitSegment) segment;

                List<TrajectoryMarker> newMarkers = new ArrayList<>(thisSegment.getMarkers());
                newMarkers.add(new TrajectoryMarker(segmentOffsetTime, marker.getCallback()));

                newSegment = new WaitSegment(thisSegment.getStartPose(), thisSegment.getDuration(), newMarkers);
            } else if (segment instanceof TurnSegment) {
                TurnSegment thisSegment = (TurnSegment) segment;

                List<TrajectoryMarker> newMarkers = new ArrayList<>(thisSegment.getMarkers());
                newMarkers.add(new TrajectoryMarker(segmentOffsetTime, marker.getCallback()));

                newSegment = new TurnSegment(thisSegment.getStartPose(), thisSegment.getTotalRotation(), thisSegment.getMotionProfile(), newMarkers);
            } else if (segment instanceof TrajectorySegment) {
                TrajectorySegment thisSegment = (TrajectorySegment) segment;

                List<TrajectoryMarker> newMarkers = new ArrayList<>(thisSegment.getTrajectory().getMarkers());
                newMarkers.add(new TrajectoryMarker(segmentOffsetTime, marker.getCallback()));

                newSegment = new TrajectorySegment(new Trajectory(thisSegment.getTrajectory().getPath(), thisSegment.getTrajectory().getProfile(), newMarkers));
            }

            sequenceSegments.set(segmentIndex, newSegment);
        }

        return sequenceSegments;
    }

    private Vector2d convertVector(Vector2d vector) {
        if (startLocation == null)
            return vector;

        return new Vector2d(vector.getX(), vector.getY() * startLocation.color);
    }

    private Pose2d convertPose(Pose2d pose) {
        if (startLocation == null)
            return pose;

        return new Pose2d(convertVector(pose.vec()), pose.getHeading() * startLocation.color);
    }

    private interface AddPathCallback {
        void run();
    }
}
