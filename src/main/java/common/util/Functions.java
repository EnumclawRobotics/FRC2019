package common.util;

public class Functions {
    // distance between two points
    public static double distance(double x1, double y1, double x2, double y2) {
        return Math.sqrt(Math.pow(x2-x1, 2d) + Math.pow(y2-y1, 2d));
    }

    // given angle 0-360
    public static double getYFromAngle(double angle) {
        return Math.cos(Math.toRadians(clip360(angle)));
    }

    // given angle 0-360
    public static double getXFromAngle(double angle) {
        return Math.sin(Math.toRadians(clip360(angle)));
    }

    // calculate gravity amount based on 0|360 being straight down
    public static double gravity(double angle) {
        return Math.sin(Math.toRadians(clip360(angle)));
    }

    // enforce 0-360 range
    public static double clip360(double angle) {
        double result = angle;
        while (result > 360) {
            result -= 360;
        }
        while (result < 0) {
            result += 360;
        }
        return result;
    }

    // enforce an arbitrary range
    public static double clip(double value, double min, double max) {
        double result = value;
        
        // NOTE: Do NOT use Math.min and Math.Max they do not handle negatives properly for our purposes
        if (value < min) {
            result = min;
        } else if (value > max) {
            result = max;
        }
        return result;
    }

    // get a slope from two points
    public static double slope(double x1, double y1, double x2, double y2) {
        return (y2-y1)/(x2-x1);
    }
}

