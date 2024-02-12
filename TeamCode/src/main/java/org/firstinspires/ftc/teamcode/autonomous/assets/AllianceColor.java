package org.firstinspires.ftc.teamcode.autonomous.assets;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;

public enum AllianceColor {
    RED(1), BLUE(-1);

    private final int multiplier;

    AllianceColor(int multiplier) {
        this.multiplier = multiplier;
    }

    /**
     * <p>Mirrors a pose based on the robot alliance.</p>
     * <p>Assumes red as the default alliance.</p>
     *
     * @param pose The pose corresponding to the red color
     * @return The mirrored pose if the alliance color is blue, the same pose otherwise
     */
    public Pose2d convertPose(Pose2d pose) {
        return new Pose2d(pose.getX(), multiplier * pose.getY(), Angle.norm(multiplier * pose.getHeading()));
    }

    public Vector2d convertVector(Vector2d vector) {
        return new Vector2d(vector.getX(), multiplier * vector.getY());
    }

    public int getMultiplier() {
        return multiplier;
    }
}
