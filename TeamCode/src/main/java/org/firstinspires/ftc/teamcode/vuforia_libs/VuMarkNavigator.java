package org.firstinspires.ftc.teamcode.vuforia_libs;

/**
 * Created by JimLori on 11/6/2016.
 */



import com.qualcomm.ftcrobotcontroller.R;
import com.vuforia.CameraDevice;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.nio.ByteBuffer;
import java.util.concurrent.BlockingQueue;


public class VuMarkNavigator {
    public static Boolean isActive = false;
    private static VuforiaLocalizer vuforia;
    private static VuforiaTrackables targets;
    private static VuforiaTrackable target;
    private static final String TARGET_ASSET_NAME = "RelicVuMark";
    private static final OpenGLMatrix TARGET_LOCATION = OpenGLMatrix.translation(0,0,0).multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC,
            AxesOrder.XYX, AngleUnit.DEGREES, 0, 0, 0));
    private static final OpenGLMatrix PHONE_LOCATION_ON_ROBOT =
            OpenGLMatrix.translation(0,0,0).multiplied(
                    Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XZX,
                            AngleUnit.DEGREES,0,0,0));;
    private static final VuforiaLocalizer.CameraDirection CAMERA_DIRECTION = VuforiaLocalizer.CameraDirection.BACK;


    public static void activate(){


        //Create the VuforiaLocalizer
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.cameraDirection = CAMERA_DIRECTION;
        parameters.vuforiaLicenseKey ="You key here.";
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        //Set up the VuforiaLocalizer to allow frame grabs of RGB565 images
        vuforia.setFrameQueueCapacity(3);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);

        //Obtain the single target and set its location (will use 0,0,0 and do all navigation relative to target position)
        targets = vuforia.loadTrackablesFromAsset(TARGET_ASSET_NAME);
        target = targets.get(0);
        target.setLocation(TARGET_LOCATION);
        ((VuforiaTrackableDefaultListener)target.getListener()).setPhoneInformation(PHONE_LOCATION_ON_ROBOT, CAMERA_DIRECTION);
        targets.activate();
        isActive = true;
    }

    //Return the RelicRecoveryVuMark code of the visualized target (RIGHT, CENTER, LEFT); UNKNOWN if no target found.
    public static RelicRecoveryVuMark getRelicRecoveryVumark(){
        return RelicRecoveryVuMark.from(target);
    }

    public static OpenGLMatrix getTargetPoseRelativeToRobot(){
        OpenGLMatrix targetPoseRelativeToCamera =
                ((VuforiaTrackableDefaultListener)target.getListener()).getPose();
        if (targetPoseRelativeToCamera == null) return null;
        return PHONE_LOCATION_ON_ROBOT.multiplied(targetPoseRelativeToCamera);
    }

    public static OpenGLMatrix getRobotPoseRelativeToTarget(){
        OpenGLMatrix targetPoseRelativeToCamera =
                ((VuforiaTrackableDefaultListener)target.getListener()).getPose();
        if (targetPoseRelativeToCamera == null) return null;
        return (PHONE_LOCATION_ON_ROBOT.multiplied(targetPoseRelativeToCamera)).inverted();
    }

    public static void setTargetLocation(OpenGLMatrix targetLocation){
        target.setLocation(targetLocation);
    }

    public static OpenGLMatrix getRobotLocation(){
        return ((VuforiaTrackableDefaultListener)target.getListener()).getRobotLocation();
    }


    //return: Z,X position and Phi, the angle between the Z axis of the coordinate system
    //being transformed to and the projection of the robot Y-axis into the ZX plane of the
    //coordinate system being transformed to.
    public static float[] getRobot_Z_X_Phi_FromLocationTransform(OpenGLMatrix locationTransform)
    {
        float[] locationData = locationTransform.getData();
        float[] returnValue = new float[3];
        returnValue[0] = locationData[14]/10.0f;
        returnValue[1] = locationData[12]/10.0f;
        returnValue[2] = (float)Math.atan2( locationData[4], locationData[6]);
        return returnValue;
    }



    public static double NormalizeAngle(double angle){
        double temp = (angle + Math.PI)/(2.0 * Math.PI);
        return (temp - Math.floor(temp) - 0.5) * 2.0 * Math.PI;
    }


    public static BlockingQueue<VuforiaLocalizer.CloseableFrame> getFrameQueue(){
        return vuforia.getFrameQueue();
    }

    public static void clearFrameQueue(BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue){
        VuforiaLocalizer.CloseableFrame tempFrame = null;
        while (true) {
            tempFrame = frameQueue.poll();
            if (tempFrame == null) break;
            tempFrame.close();
        }
    }

    public static boolean getRGB565Array(BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue, int width, int height, byte[] dst) {
        if (dst.length != (2 * width * height)) return false;
        VuforiaLocalizer.CloseableFrame frame = null;
        VuforiaLocalizer.CloseableFrame tempFrame = null;
        Image img = null;
        try{
            //We want the most recent available frame, which necessitates this while loop. If no frame is available, return false.
            while (true){
                tempFrame = frameQueue.poll();
                if (tempFrame == null) break;
                if (frame != null) frame.close();
                frame = tempFrame;
            }
            if (frame == null) return false;

            //Iterate through the images in the frame to find one that satisfies the width, height, and pixel format requirements
            long numImages = frame.getNumImages();
            for (int i = 0; i < numImages; i++){
                img = frame.getImage(i);
                if (img.getFormat() == PIXEL_FORMAT.RGB565 && img.getWidth() == width && img.getHeight() == height){
                    ByteBuffer byteBuf = img.getPixels();
                    byteBuf.get(dst);
                    return true;
                }
            }
            return false;
        }
        finally{
            if (frame != null) frame.close();
            if (tempFrame != null) tempFrame.close();
        }
    }

    //Turn camera flashlight on or off
    public static boolean setFlashTorchMode(boolean on){
        return CameraDevice.getInstance().setFlashTorchMode(on);
    }
}
