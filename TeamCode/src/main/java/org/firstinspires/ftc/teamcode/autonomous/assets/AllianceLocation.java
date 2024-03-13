package org.firstinspires.ftc.teamcode.autonomous.assets;

import android.util.Pair;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public enum AllianceLocation {
    RED_SHORT(1, 1), BLUE_SHORT(-1, 1), RED_LONG(1, -1), BLUE_LONG(-1, -1);

    public final int color, side;

    AllianceLocation(int color, int side) {
        this.color = color;
        this.side = side;
    }

    /**
     * Gets the visible cases of the camera during TensorFlow object detection.
     *
     * @return A pair of cases, with each member representing the visible case in its respective half
     */
    public Pair<PropLocations, PropLocations> getVisibleCases() {
        if (Math.signum(color * side) == 1)
            return new Pair<>(PropLocations.MIDDLE, PropLocations.RIGHT);
        else return new Pair<>(PropLocations.LEFT, PropLocations.MIDDLE);
    }

    /**
     * Gets the case invisible to the camera during TensorFlow object detection.
     */
    public PropLocations getHiddenCase() {
        return Math.signum(color * side) == 1 ? PropLocations.LEFT : PropLocations.RIGHT;
    }

    /**
     * @return A pose containing the starting coordinates of the robot
     */
    public Pose2d getStartingPosition() {
        double x, y = -62.75, theta = 90.0;
        x = side == 1 ? 16.75 : -40.25;

        return new Pose2d(x, color * y, Math.toRadians(color * theta));
    }
}
