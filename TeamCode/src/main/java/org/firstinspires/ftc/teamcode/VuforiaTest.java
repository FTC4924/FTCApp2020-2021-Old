package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="VuforiaTest")
public class VuforiaTest extends OpMode {

    int cameraMonitorViewId;

    VuforiaLocalizer vuforia;
    VuforiaLocalizer.Parameters parameters;
    VuforiaTrackables skystoneTrackables;
    List<VuforiaTrackable> allTrackables;

    public void init() {

        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = " -- YOUR NEW VUFORIA KEY GOES HERE  --- ";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        skystoneTrackables = this.vuforia.loadTrackablesFromAsset("Skystone");
        allTrackables = new ArrayList<>(skystoneTrackables);

        skystoneTrackables.activate();

    }

    public void loop() {
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {

                telemetry.addData("Visible Target", trackable.getName());

                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)trackable.getListener()).getPose();
                telemetry.addData("Pose", format(pose));

            }
        }

        telemetry.update();

    }

    String format(OpenGLMatrix transformationMatrix) {

        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";

    }
}
