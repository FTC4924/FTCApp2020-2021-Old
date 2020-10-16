package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@TeleOp(name="NerfRobotSteering")
public class NerfRobotSteering extends OpMode {

    WebcamName webcamName = null;

    BNO055IMU imu;
    Orientation angles;
    BNO055IMU.Parameters parameters;

    private DcMotor rightMotor;
    private DcMotor leftMotor;

    public double neededAngle;
    public double currentAngle;

    public double error;
    public double lastError;
    public double integral;
    public double derivative;

    public static final double KP = .005;
    public static final double KI = 0;
    public static final double KD = 0;

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_LANDSCAPE = true;

    private static final String VUFORIA_KEY = "AaeF/Hb/////AAABmXyUA/dvl08Hn6O8IUco1axEjiRtYCVASeXGzCnFiMaizR1b3cvD+SXpU1UHHbSpnyem0dMfGb6wce32IWKttH90xMTnLjY4aXBEYscpQbX/FzUi6uf5M+sXDVNMtaVxLDGOb1phJ8tg9/Udb1cxIUCifI+AHmcwj3eknyY1ZapF81n/R0mVSmuyApS2oGQLnETWaWK+kxkx8cGnQ0Nj7a79gStXqm97obOdzptw7PdDNqOfSLVcyKCegEO0zbGoInhRMDm0MPPTxwnBihZsjDuz+I5kDEZJZfBWZ9O1PZMeFmhe6O8oFwE07nFVoclw7j2P6qHbsKTabg3w9w4ZdeTSZI4sV2t9OhbF13e0MWeV";

    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;

    private static final float stoneZ = 2.00f * mmPerInch;

    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;
    private static final float bridgeRotZ = 180;

    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    OpenGLMatrix robotFromCamera;

    final float CAMERA_FORWARD_DISPLACEMENT  = 0;
    final float CAMERA_VERTICAL_DISPLACEMENT = 0;
    final float CAMERA_LEFT_DISPLACEMENT     = 0;

    private OpenGLMatrix lastLocation;
    private VuforiaLocalizer vuforia;
    private boolean targetVisible;
    private float phoneXRotate;
    private float phoneYRotate;
    private float phoneZRotate;

    int cameraMonitorViewId;
    VuforiaLocalizer.Parameters vuforiaParameters;

    VuforiaTrackables targetsSkyStone;

    VuforiaTrackable stoneTarget;
    VuforiaTrackable blueRearBridge;
    VuforiaTrackable redRearBridge;
    VuforiaTrackable redFrontBridge;
    VuforiaTrackable blueFrontBridge;
    VuforiaTrackable red1;
    VuforiaTrackable red2;
    VuforiaTrackable front1;
    VuforiaTrackable front2;
    VuforiaTrackable blue1;
    VuforiaTrackable blue2;
    VuforiaTrackable rear1;
    VuforiaTrackable rear2;

    List<VuforiaTrackable> allTrackables;

    private DcMotor middleMotor;
    private DcMotor rightMotor;
    private DcMotor leftMotor;

    private double middlePower;
    private double leftPower;
    private double rightPower;

    private double gamepad1LeftStickY;
    private double gamepad1LeftStickX;
    private double gamepad1RightStickX;
    private double gamepad1RightStickY;

    @Override
    public void init() {

        middleMotor = hardwareMap.get(DcMotor.class, "middleWheelMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);



    }

    @Override
    public void loop() {

        middlePower = 0;
        leftPower = 0;
        rightPower = 0;

        gamepad1LeftStickX = gamepad1.left_stick_x;
        gamepad1LeftStickY = gamepad1.left_stick_y;
        gamepad1RightStickX = gamepad1.right_stick_x / 2;
        gamepad1RightStickY = gamepad1.right_stick_y;

        if(gamepad1LeftStickX > 0.05 || gamepad1LeftStickX < -.05) {

            middlePower = gamepad1LeftStickX;
        }

        if(gamepad1LeftStickY > 0.05 || gamepad1LeftStickY < -.05) {

            leftPower = gamepad1LeftStickY;
            rightPower = gamepad1LeftStickY;

        }

        if(gamepad1RightStickX > 0.05){
            if(leftPower > 1 - gamepad1RightStickX) {

                leftPower -= rightPower + gamepad1RightStickX - 1 + gamepad1RightStickX;
                rightPower = 1;

            } else if(leftPower < -1 + gamepad1RightStickX) {

                rightPower -= leftPower - gamepad1RightStickX + 1 - gamepad1RightStickX;
                leftPower = -1;

            } else {

                rightPower += gamepad1RightStickX;
                leftPower -= gamepad1RightStickX;

            }
        } else if(gamepad1RightStickX < -0.05) {
            if(leftPower > 1 + gamepad1RightStickX) {

                rightPower -= leftPower - gamepad1RightStickX - 1 - gamepad1RightStickX;
                leftPower = 1;

            } else if(leftPower < -1 - gamepad1RightStickX) {

                leftPower -= rightPower + gamepad1RightStickX + 1 + gamepad1RightStickX;
                rightPower = -1;

            } else {

                leftPower -= gamepad1RightStickX;
                rightPower += gamepad1RightStickX;

            }
        }

        middleMotor.setPower(middlePower);
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
        telemetry.addData("middlePower", middlePower);
        telemetry.addData("leftPower", leftPower);
        telemetry.addData("rightPower", rightPower);

        if (gamepad1.a) {



        }
    }
}