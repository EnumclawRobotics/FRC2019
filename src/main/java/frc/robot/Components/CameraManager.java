// package frc.robot.Components;

// import org.opencv.core.Mat;
// import org.opencv.core.Point;
// import org.opencv.core.Scalar;
// import org.opencv.imgproc.Imgproc;

// import edu.wpi.cscore.UsbCamera;
// import edu.wpi.cscore.CvSink;
// import edu.wpi.cscore.CvSource;
// import edu.wpi.cscore.MjpegServer;
// import edu.wpi.cscore.VideoMode.PixelFormat;

// import common.instrumentation.Telemetry;
// import frc.robot.RobotMap;

// public class CameraManager {
//     private Telemetry telemetry = new Telemetry("Robot/CameraManager");

//     private static UsbCamera cameraNormal; 
//     private static UsbCamera cameraInverted; 

//     private static MjpegServer mjpegServerNormal;
//     private static CvSink cvSinkNormal;
//     private static MjpegServer mjpegServerInverted;
//     private static CvSink cvSinkInverted;
//     private static MjpegServer mjpegServerFacing;
//     private static CvSource cvFacing;

//     public static boolean facingNormal;

//     public CameraManager(RobotMap robotMap) {
//         cameraNormal = robotMap.cameraNormal;
//         cameraInverted = robotMap.cameraInverted;

//         mjpegServerNormal = new MjpegServer("serve_USB Camera 0", 1181);
//         mjpegServerNormal.setSource(cameraNormal); 
//         cvSinkNormal = new CvSink("opencv_USB Camera 0");
//         cvSinkNormal.setSource(cameraNormal);

//         mjpegServerInverted = new MjpegServer("serve_USB Camera 1", 1181);
//         mjpegServerInverted.setSource(cameraInverted); 
//         cvSinkInverted = new CvSink("opencv_USB Camera 1");
//         cvSinkInverted.setSource(cameraInverted);

//         cvFacing = new CvSource("Facing Camera", PixelFormat.kMJPEG, 640, 480, 30);
//         mjpegServerFacing = new MjpegServer("serve_Facing", 1182);
//         mjpegServerFacing.setSource(cvFacing);
//     }
    
//     public void init() {
//         new Thread(() -> {
//             Mat source = new Mat();
            
//             while(!Thread.interrupted()) {
//                 // get a frame as input
//                 if (facingNormal) {
//                     cvSinkNormal.grabFrame(source);
//                 } else {
//                     cvSinkInverted.grabFrame(source);
//                 }

//                 // draw two center lines
//                 double centerX = source.width()/2d;
//                 Imgproc.line(source, new Point(centerX-2, 0), new Point(centerX+2, source.height()), new Scalar(0,255,0), 2);

//                 // put a frame to output
//                 cvFacing.putFrame(source);
//             }

//         }).start();

//     }

//     public void setFacing(boolean facingNormal) {
//         if (CameraManager.facingNormal != facingNormal) {
//             CameraManager.facingNormal = facingNormal;
//         }
//     }

//     public void stop() {
//     }

//     public void run() {
//         putTelemetry();
//     }

//     public void putTelemetry() {

//     }

// }