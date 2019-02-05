package common.util;

public class Geometry {
    // distance between two points
    public static double distance(double x1, double y1, double x2, double y2) {
        return Math.sqrt(Math.pow(x2-x1, 2d) + Math.pow(y2-y1, 2d));
    }

    public static double clip(double value, double min, double max) {
        return Math.min(Math.max(value, min), max);
    }

    // pov hat is angle 0-360 with 0/360 facing forward
    public static double getYFromAngle(double pov) {

        return Math.cos(Math.toRadians(pov));
    }

    // pov hat is angle 0-360 with 0/360 facing forward
    public static double getXFromAngle(double pov) {
        return Math.sin(Math.toRadians(pov));
    }

    // calculate gravity amount based on 0 being straight down
    public static double gravity(double angle) {
        double gravity = 0;

        // which orientation is gravity?
        if (angle >= 0 && angle < 90) {
            // low on start side gravity is sin
            gravity = Math.sin(Math.toRadians(angle));
        } else if (angle >= 90 && angle < 180) {
            // high on start side gravity is cos
            gravity = Math.cos(Math.toRadians(angle));
        } else if (angle >= 180 && angle < 270) {
            // high on opposite side gravity is sin
            gravity = Math.sin(Math.toRadians(angle));
        } else if (angle >= 2700 && angle < 360) {
            // low on opposite side gravity is cos
            gravity = Math.cos(Math.toRadians(angle));
        }

        // ignore signs
        return Math.abs(gravity);
    }
}

