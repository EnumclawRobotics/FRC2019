package frc.robot.Components;

import common.pixy2Api.*;

import edu.wpi.first.cameraserver.CameraServer;
import frc.robot.RobotMap;

public class CameraManager {
    private Pixy2 pixy2Normal;
    private Pixy2 pixy2Inverted;
    private boolean facingNormal;
    private int cameraNormalUsb;
    private int cameraInvertedUsb;

    public CameraManager(RobotMap robotMap) {
        this.pixy2Normal = robotMap.pixy2Normal;
        this.pixy2Inverted = robotMap.pixy2Inverted; 
        this.facingNormal = true;

        // start line tracking program
        this.pixy2Normal.getLine().setMode(Pixy2Line.LINE_MODE_WHITE_LINE);
        this.pixy2Inverted.getLine().setMode(Pixy2Line.LINE_MODE_WHITE_LINE);
    }
    
    public void setFacing(boolean facingNormal) {
        this.facingNormal = facingNormal;
        CameraServer.getInstance().startAutomaticCapture("", cameraNormalUsb);
    }

    private Pixy2 getPixy() {
        return (facingNormal ? pixy2Normal : pixy2Inverted);
    } 

    public Vector getVector() {
        Vector result = null;

        Pixy2 pixy2 = getPixy();
        Pixy2Line pixy2Line = pixy2.getLine();
        pixy2Line.getFeatures(Pixy2Line.LINE_GET_MAIN_FEATURES, Pixy2Line.LINE_VECTOR, true); 
        Vector[] vectors = pixy2Line.getVectors(); 

        if (vectors.length > 0) {
            result = vectors[0];
        }

        return result;
    }

    // // expects form "vector: a,b - c,d"
    // public static Point2D.Double[] convertVector(String vector) {
    //     Point2D.Double[] result = null;

    //     if (!vector.isEmpty()) {
    //         vector = vector.replace("vector: ", "");
    //         String[] points = vector.split(" - ", 0);

    //         if (points.length == 2) {
    //             Point2D.Double startPoint = convertPoint(points[0]);
    //             Point2D.Double endPoint = convertPoint(points[1]);

    //             if (startPoint != null && endPoint != null) {
    //                 result = new Point2D.Double[] { startPoint, endPoint};
    //             }
    //         }
    //     }

    //     return result;
    // }

    // // expects form "a,b"
    // public static Point2D.Double convertPoint(String point) {
    //     Point2D.Double result = null;

    //     String[] values = point.split(",", 0);
    //     if (values.length == 2) {
    //         Double x = Double.valueOf(values[0]);
    //         Double y = Double.valueOf(values[1]);
    //         if (!x.isNaN() && !y.isNaN()) {
    //             result = new Point2D.Double(x, y);
    //         }
    //     } 

    //     return result;
    // } 
}