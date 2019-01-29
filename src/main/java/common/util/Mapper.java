package common.util;

import java.awt.Point;

public class Mapper {
    double cameraHeight;            // [Height from floor];
    double cameraPitch;             //[Angle of depression];
    double cameraFieldOfView;       // [If 1:1 aspect ratio, use this]; 
    //Pixy2 color object resolution: 316-208, field of view is 60-40, both of which ~3:2 ratio

    public Mapper(double cameraHeight, double cameraPitch, double cameraFieldOfView) {
        this.cameraHeight = cameraHeight;
        this.cameraPitch = cameraPitch;
        this.cameraFieldOfView = cameraFieldOfView;
    }

    /**
     * converts object coordinates seen by the Pixy2 into floor coordinates relative to the camera
     * viewPortLocation [0-1 Percent of object detection point];
     * NOTE: PIXY docs suggest white line endpoints will be in a 0-75 by 0-59 grid. 
     */
    public Point PixyToFieldCoords(Point viewPortLocation) {
        double verticalAngleOffset = cameraFieldOfView * (viewPortLocation.y - 0.5d);
        double horizontalAngleOffset = cameraFieldOfView * (viewPortLocation.x - 0.5d);

        double forwardTranslation = cameraHeight * Math.tan(Math.toRadians(verticalAngleOffset + cameraPitch - 90));

        double sidewaysTranslation = -Math.sin(Math.toRadians(cameraPitch)) * (cameraHeight * Math.tan(Math.toRadians(horizontalAngleOffset)));

        sidewaysTranslation += Math.cos(Math.toRadians(cameraPitch)) * ((Math.PI/2) * (viewPortLocation.getX() - 0.5d) * forwardTranslation * Math.tan(Math.toRadians(cameraFieldOfView / 2)));

        return new Point((int)sidewaysTranslation, (int)forwardTranslation);
    }

    /**
    *
    Curve generation from this discussion: https://www.chiefdelphi.com/t/curve-driving-generator/121051/10

    Has this math to create a curve from two segments, ie sets of points, or two points each with a tangent slope or a combo approach
    see the matrix math solution instad of the algebraic for less errata conditions.

    https://www.chiefdelphi.com/uploads/default/original/3X/1/1/1199b2afee08adbaae2a861c18c7fdf3804facc0.pdf 

    a=-(-2y2+2y1+(m2+m1)x2+(-m2-m1)x1)/(-x2^3+3x1x2^2-3x1^2x2+x1^3);

    b=(-3x2y2+x1*((m2-m1)x2-3y2)+(3x2+3x1)y1+(m2+2m1)x2^2+(-2m2-m1)x1^2)/(-x2^3+3x1x2^2-3x1^2*x2+x1^3);

    c=-(x1*((2m2+m1)x2^2-6x2y2)+6x1x2y1+m1x2^3+(-m2-2m1)x1^2x2-m2x1^3)/(-x2^3+3x1x2^2-3x1^2x2+x1^3);

    d=(x1^2*((m2-m1)x2^2-3x2y2)+x1^3(y2-m2x2)+(3x1x2^2-x2^3)y1+m1x1x2^3)/(-x2^3+3x1x2^2-3x1^2x2+x1^3);
    */

}