package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

@TeleOp(name="VuforiaTest")
public class VuforiaTest extends OpMode {

    int cameraMonitorViewId;

    VuforiaLocalizer vuforia;
    VuforiaLocalizer.Parameters parameters;
    VuforiaTrackables skystoneTrackables;
    List<VuforiaTrackable> allTrackables;

    private static final float mmPerInch        = 25.4f;

    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    public void init() {

        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AaeF/Hb/////AAABmXyUA/dvl08Hn6O8IUco1axEjiRtYCVASeXGzCnFiMaizR1b3cvD+SXpU1UHHbSpnyem0dMfGb6wce32IWKttH90xMTnLjY4aXBEYscpQbX/FzUi6uf5M+sXDVNMtaVxLDGOb1phJ8tg9/Udb1cxIUCifI+AHmcwj3eknyY1ZapF81n/R0mVSmuyApS2oGQLnETWaWK+kxkx8cGnQ0Nj7a79gStXqm97obOdzptw7PdDNqOfSLVcyKCegEO0zbGoInhRMDm0MPPTxwnBihZsjDuz+I5kDEZJZfBWZ9O1PZMeFmhe6O8oFwE07nFVoclw7j2P6qHbsKTabg3w9w4ZdeTSZI4sV2t9OhbF13e0MWeV";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        skystoneTrackables = this.vuforia.loadTrackablesFromAsset("Skystone");

        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        VuforiaTrackable stoneTarget = skystoneTrackables.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = skystoneTrackables.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = skystoneTrackables.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = skystoneTrackables.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = skystoneTrackables.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = skystoneTrackables.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = skystoneTrackables.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = skystoneTrackables.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = skystoneTrackables.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = skystoneTrackables.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = skystoneTrackables.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = skystoneTrackables.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = skystoneTrackables.get(12);
        rear2.setName("Rear Perimeter 2");

        allTrackables = new ArrayList<>(skystoneTrackables);

        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        skystoneTrackables.activate();

    }

    public void loop() {
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {

                telemetry.addData("Visible Target", trackable.getName());

                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                telemetry.addData("Pose", format(pose));

            }
        }

        telemetry.update();

    }

    String format(OpenGLMatrix transformationMatrix) {

        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";

    }
}
