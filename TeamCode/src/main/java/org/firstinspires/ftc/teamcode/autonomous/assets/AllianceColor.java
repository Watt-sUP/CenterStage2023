package org.firstinspires.ftc.teamcode.autonomous.assets;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;

public enum AllianceColor {
    RED(1), BLUE(-1);

    private final double multiplier;

    AllianceColor(double multiplier) {
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
        return new Pose2d(Math.abs(multiplier) * pose.getX(), Math.signum(multiplier) * pose.getY(), Angle.norm(Math.signum(multiplier) * pose.getHeading()));
    }
}
