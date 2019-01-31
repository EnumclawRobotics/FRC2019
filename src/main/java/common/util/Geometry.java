package common.util;

public class Geometry {
    // distance between two points
    public static double distance(double x1, double y1, double x2, double y2) {
        return Math.sqrt(Math.pow(x2-x1, 2d) + Math.pow(y2-y1, 2d));
    }

}