package org.firstinspires.ftc.teamcode.autonomous.assets;

import androidx.annotation.NonNull;

public enum PropLocations {
    LEFT, MIDDLE, RIGHT;

    @NonNull
    @Override
    public String toString() {
        String name = this.name();
        return name.substring(0, 1).toUpperCase() + name.substring(1).toLowerCase();
    }
}
