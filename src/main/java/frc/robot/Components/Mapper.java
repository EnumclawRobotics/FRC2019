package frc.robot.Components;

import common.util.*;
import common.instrumentation.Telemetry;
import common.pixy2Api.*;
import frc.robot.RobotMap;

import java.awt.geom.Point2D;

public class Mapper {
    Telemetry telemetry = new Telemetry("Robot/Mapper");

    private Pixy2 pixy2Normal;
    private Pixy2 pixy2Inverted;
    private boolean facingNormal;
    private Vector vector;
    private double rotation; 

    private double cameraElevation;         // Camera height from floor;
    private double cameraAngle;             // Angle between vertical and camera facing in degrees;
    private double aspectX;                 // X pixel to X FOV conversion
    private double aspectY;                 // X pixel to Y FOV conversion
    private double centerX;                 // center coordinate
    private double centerY;                 // center coordinate

    //Pixy2 color object resolution: 316-208, field of view is 60-40, both of which ~3:2 ratio
    public Mapper(RobotMap robotMap) {
        this.pixy2Normal = robotMap.pixy2Normal;
        this.pixy2Inverted = robotMap.pixy2Inverted;
 
        // start line tracking program - uses a 0-79, 0-59 grid
        this.pixy2Normal.getLine().setMode(Pixy2Line.LINE_MODE_WHITE_LINE);
        this.pixy2Inverted.getLine().setMode(Pixy2Line.LINE_MODE_WHITE_LINE);

        // face forward at start
        facingNormal = true;
 
        // get the settings right so we can translate coords
        this.cameraElevation = robotMap.cameraElevation;
        this.cameraAngle = robotMap.cameraAngle;
        this.aspectX = robotMap.cameraFovX / robotMap.cameraMaxX;
        this.aspectY = robotMap.cameraFovY / robotMap.cameraMaxY;
        this.centerX = robotMap.cameraMaxX * .5d;
        this.centerY = robotMap.cameraMaxY * .5d;
    }

    public boolean getFacing() {
        return facingNormal;
    }

    public void setFacing(boolean facingNormal) {
        this.facingNormal = facingNormal;
    }

    // current pixy to listen to
    private Pixy2 getPixy() {
        return (facingNormal ? pixy2Normal : pixy2Inverted);
    } 

    // do we have a main white line?
    public Vector getVector() {
        vector = null;

        Pixy2 pixy2 = getPixy();
        Pixy2Line pixy2Line = pixy2.getLine();
        pixy2Line.getFeatures(Pixy2Line.LINE_GET_MAIN_FEATURES, Pixy2Line.LINE_VECTOR, true); 
        Vector[] vectors = pixy2Line.getVectors(); 

        if (vectors.length > 0) {
            vector = vectors[0];
        }

        return vector;
    }

    // figure out relative floor coords of white line seen by Pixy2
    public Vector CameraVectorToFieldVector(Vector vector) {
        Point2D.Double start = CameraPointToFieldPoint(vector.getX0(), vector.getY0());
        Point2D.Double arrow = CameraPointToFieldPoint(vector.getX1(), vector.getY1());

        return new Vector(start.x, start.y, arrow.x, arrow.y, vector.getIndex(), vector.getFlags());
    }

    /**
     * figure out relative floor coord of a coord seen by Pixy2
     */
    public Point2D.Double CameraPointToFieldPoint(double x, double y) {
        double xAngle;
        double yAngle;
        double forward;
        double sideways;

        // translate the point so that center of screen (vanishing point) is at 0,0 coords   
        x = (x - centerX);
        y = (y - centerY);

        // get angles represented in the view coordinates
        xAngle = x * aspectX;
        yAngle = y * aspectY;

        // get distance in front of bot - note yangle is offset to camera angle 
        forward = cameraElevation / Math.tan(Math.toRadians(cameraAngle + yAngle));
        
        // get distance sideways - note xangle is signed for left and right   
        sideways = forward / Math.tan(Math.toRadians(xAngle));
       
        // not a perfect translation but good enough approximation for us 
        return new Point2D.Double(sideways, forward);
    }

    // given floor coords of a white line and current speed 
    // generate a rotation value to help the bot line up on the white line when it gets there
    public static double getRotation(Vector vector, double speed) {
        // algorithm:
        // assume bot is facing along zero axis and the white line coordinates use the same reference
        // get the slope of white line starting at pint2 and going to point3
        // calculate the function that will describe a curve to end up aligned with the white line slope when at point2
        // pick a waypoint xw spot just ahead based on speed and use the xw as an input into the function above and get the yw out
        // figure out the rotation angle needed described by the arctan of yw/xw
        // divide the angle by 90 to get into the range = [-1,1]
        // return this rotation to try and follow the path
        // repeat each clock cycle with updated data

        /**
        *
        Curve generation from this discussion: https://www.chiefdelphi.com/t/curve-driving-generator/121051/10

        Has this math to create a curve from two segments, ie sets of points, or two points each with a tangent slope or a combo approach
        see the matrix math solution instad of the algebraic for less errata conditions.

        https://www.chiefdelphi.com/uploads/default/original/3X/1/1/1199b2afee08adbaae2a861c18c7fdf3804facc0.pdf 

        a=-(-2y2+2y1+(m2+m1)x2+(-m2-m1)x1)/(-x2^3+3x1x2^2-3x1^2x2+x1^3);
        b=(-3x2y2+x1((m2-m1)x2-3y2)+(3x2+3x1)y1+(m2+2m1)x2^2+(-2m2-m1)x1^2)/(-x2^3+3x1x2^2-3x1^2x2+x1^3);
        c=-(x1((2m2+m1)x2^2-6x2y2)+6x1x2y1+m1x2^3+(-m2-2m1)x1^2x2-m2x1^3)/(-x2^3+3x1x2^2-3x1^2x2+x1^3);
        d=(x1^2((m2-m1)x2^2-3x2y2)+x1^3(y2-m2x2)+(3x1x2^2-x2^3)y1+m1x1x2^3)/(-x2^3+3x1x2^2-3x1^2x2+x1^3);

        but if x1,y1,m1 are 0 then

        a=(m2x2-2y2)/(x2^3);
        b=(3y2-m2x2)/(x2^2)
        c=0;
        d=0;

        so yw(xw) = a(xw)^3 + b(xw)^2

        */

        double m2 = Geometry.slope(vector.getX0(), vector.getY0(), vector.getX1(), vector.getY1());

        double a = (m2 * vector.getX0() - 2 * vector.getY0())/Math.pow(vector.getX0(), 3);
        double b = (3 * vector.getY0() - m2 * vector.getX0())/Math.pow(vector.getX0(), 2);

        // planning forward waypoint at 60 cycles/sec. full speed is 50"/sec  
        double xw = 50d/60d * speed;
        double yw = a * Math.pow(xw, 3) + b * Math.pow(xw, 2);

        // rotation needed where range -90:90 mapped to -1:1
        double rotation = Math.toDegrees(Math.atan(yw / xw)) / 90d;

        return rotation;
    }

    public void stop() {
    }

    public void run() {
        putTelemetry();
    }

    public void putTelemetry() {
        telemetry.putDouble("Rotation", rotation);
        telemetry.putString("Version", "1.0.0");
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

