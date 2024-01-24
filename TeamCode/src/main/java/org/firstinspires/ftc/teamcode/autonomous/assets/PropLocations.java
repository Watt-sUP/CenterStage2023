package org.firstinspires.ftc.teamcode.autonomous.assets;

public enum PropLocations {
    LEFT(-1), MIDDLE(0), RIGHT(1);

    private final int id;

    PropLocations(int id) {
        this.id = id;
    }

    public static PropLocations fromId(int id) throws IllegalArgumentException {
        for (PropLocations location : PropLocations.values())
            if (location.id == id)
                return location;

        throw new IllegalArgumentException("No matching location for ID: " + id);
    }

    public int getId() {
        return id;
    }
}
