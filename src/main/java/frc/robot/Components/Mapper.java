package frc.robot.Components;

import common.util.*;
import frc.robot.RobotMap;

import java.awt.geom.Point2D;

public class Mapper {
    private double cameraElevation;         // Camera height from floor;
    private double cameraAngle;             // Angle between vertical and camera facing in degrees;
    private double aspectX;                 // X pixel to X FOV conversion
    private double aspectY;                 // X pixel to Y FOV conversion
    private double centerX;                 // center coordinate
    private double centerY;                 // center coordinate

    //Pixy2 color object resolution: 316-208, field of view is 60-40, both of which ~3:2 ratio
    public Mapper(RobotMap robotMap) {
        this.cameraElevation = robotMap.cameraElevation;
        this.cameraAngle = robotMap.cameraAngle;
        this.aspectX = robotMap.cameraFovX / robotMap.cameraMaxX;
        this.aspectY = robotMap.cameraFovY / robotMap.cameraMaxY;
        this.centerX = robotMap.cameraMaxX * .5d;
        this.centerY = robotMap.cameraMaxY * .5d;
    }

    /**
     * converts object coordinates seen by the Pixy2 into floor coordinates relative to the camera
     * viewPortLocation [0-1 Percent of object detection point];
     * NOTE: PIXY docs suggest white line endpoints will be in a 0-75 by 0-59 grid. 
     */
    public Point2D.Double CameraPointToFieldPoint(Point2D.Double viewPortPoint) {
        double x = viewPortPoint.x;
        double y = viewPortPoint.y;
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

    // given start and end coords of a white line and current speed 
    // generate a rotation value to help the bot line up on the white line
    public static double getRotation(Point2D.Double point2, Point2D.Double point3, double speed) {
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

        double m2 = Geometry.slope(point2.x, point2.y, point3.x, point3.y);

        double a = (m2 * point2.x - 2 * point2.y)/Math.pow(point2.x, 3);
        double b = (3 * point2.y - m2 * point2.x)/Math.pow(point2.x, 2);

        // planning forward waypoint at 60 cycles/sec. full speed is 50"/sec  
        double xw = 50d/60d * speed;
        double yw = a * Math.pow(xw, 3) + b * Math.pow(xw, 2);

        // rotation needed where range -90:90 mapped to -1:1
        double rotation = Math.toDegrees(Math.atan(yw / xw)) / 90d;

        return rotation;
    }
}