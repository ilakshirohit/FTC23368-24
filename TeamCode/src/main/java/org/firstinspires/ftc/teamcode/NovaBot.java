package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.tfod.TfodProcessor; // ?????? change object detetcion import or usage

public class NovaBot {

    public int count = 0;

    public DcMotor leftSliderMotor;
    public DcMotor rightSliderMotor;
    public DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    public DcMotor armMotor; // change - up/dowm movement
    public Servo claw;

    public IMU imu;
    YawPitchRollAngles robotOrientation;
    public TouchSensor limitSwitch; // after design of slides, decide whether to keep
    public boolean isSliderMoving = false;
    public boolean slidersResetByLimitSwitch = false;

    public ElapsedTime runtime = new ElapsedTime();


    public static final double ENCODER_TICKS_PER_INCH = (3009/69);
    public static final double STRAFING_ENCODER_TICKS_PER_INCH = (50.75);

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "NovaTeamPropModel23.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    //private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "blue", "red", "none"
    };

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
//    public TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    public VisionPortal visionPortal;

    public LinearOpMode linearOpMode;
    public HardwareMap hardwareMap;

    public NovaBot(LinearOpMode callingLinearOpMode) {
        this.linearOpMode = callingLinearOpMode;
        this.hardwareMap = callingLinearOpMode.hardwareMap;
    }


    public void initNovaBot() {
//        initTfod();

        // Wait for the DS start button to be touched.
        linearOpMode.telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        linearOpMode.telemetry.addData(">", "Touch Play to start OpMode");
        linearOpMode.telemetry.update();

        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        robotOrientation = imu.getRobotYawPitchRollAngles();

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armMotor = hardwareMap.dcMotor.get("armMotor"); // stays the same
        claw = hardwareMap.servo.get("claw");

        limitSwitch = hardwareMap.touchSensor.get("limitSwitch"); // ??

        leftSliderMotor = hardwareMap.dcMotor.get("leftSliderMotor");
        rightSliderMotor = hardwareMap.dcMotor.get("rightSliderMotor");
        rightSliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // These lines reset the encoder and do the job of the limit switch method
        leftSliderMotor.setPower(0);
        rightSliderMotor.setPower(0);

        leftSliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSliderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSliderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        linearOpMode.telemetry.addData("Status: ", "Robot Initialized");
        linearOpMode.telemetry.update();
    }

//    private void initTfod() {
//
//        // Create the TensorFlow processor by using a builder.
//        tfod = new TfodProcessor.Builder()
//
//                // With the following lines commented out, the default TfodProcessor Builder
//                // will load the default model for the season. To define a custom model to load,
//                // choose one of the following:
//                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
//                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.\
//                .setModelAssetName(TFOD_MODEL_ASSET)
//                //.setModelFileName(TFOD_MODEL_FILE)
//
//                // The following default settings are available to un-comment and edit as needed to
//                // set parameters for custom models.
//                .setModelLabels(LABELS)
//                .setIsModelTensorFlow2(true)
//                .setIsModelQuantized(true)
//                .setModelInputSize(480)
//                .setModelAspectRatio(16.0 / 9.0)
//
//                .build();
//
//        // Create the vision portal by using a builder.
//        VisionPortal.Builder builder = new VisionPortal.Builder();
//
//        // Set the camera (webcam vs. built-in RC phone camera).
//        if (USE_WEBCAM) {
//            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
//        } else {
//            builder.setCamera(BuiltinCameraDirection.BACK);
//        }
//
//        // Choose a camera resolution. Not all cameras support all resolutions.
//        //builder.setCameraResolution(new Size(640, 480));
//
//        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
//        builder.enableLiveView(true);
//
//        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
//        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
//
//        // Choose whether or not LiveView stops if no processors are enabled.
//        // If set "true", monitor shows solid orange screen if no processors enabled.
//        // If set "false", monitor shows camera view without annotations.
//        //builder.setAutoStopLiveView(false);
//
//        // Set and enable the processor.
//        builder.addProcessor(tfod);
//
//        // Build the Vision Portal, using the above settings.
//        visionPortal = builder.build();
//
//        // Set confidence threshold for TFOD recognitions, at any time.
//        tfod.setMinResultConfidence(0.75f);
//
//        // Disable or re-enable the TFOD processor at any time.
//        //visionPortal.setProcessorEnabled(tfod, true);
//
//    }   // end method initTfod()

    public void gyroTurnRight(double targetAngle) {
        imu.resetYaw();
        double currentHeading = getGyroAngle();

        while (this.linearOpMode.opModeIsActive() && currentHeading > targetAngle) {
            currentHeading = getGyroAngle();

            this.linearOpMode.telemetry.addData("currentHeading: ", getAngle());
            this.linearOpMode.telemetry.update();

            frontLeftMotor.setPower(-0.3);
            backLeftMotor.setPower(-0.3);
            frontRightMotor.setPower(0.3);
            backRightMotor.setPower(0.3);
        }

        this.linearOpMode.telemetry.addData("Current Angle ", getAngle());
        this.linearOpMode.telemetry.update();

        this.linearOpMode.telemetry.addData("Status: ", " exited while loop");
        this.linearOpMode.telemetry.update();

        stopAllMotors();
    }

    public void gyroTurnLeft(double targetAngle) {
        count++;

        linearOpMode.telemetry.addData("Status", "entered gyroTurnLeft");
        linearOpMode.telemetry.update();

        imu.resetYaw();
        double currentHeading = getGyroAngle();

        while (this.linearOpMode.opModeIsActive() && currentHeading < targetAngle) {
            currentHeading = getGyroAngle();

            this.linearOpMode.telemetry.addData("currentHeading: ", getAngle());
            this.linearOpMode.telemetry.update();

            frontLeftMotor.setPower(0.3);
            backLeftMotor.setPower(0.3);
            frontRightMotor.setPower(-0.3);
            backRightMotor.setPower(-0.3);
        }

        this.linearOpMode.telemetry.addData("Current Angle ", getAngle());
        this.linearOpMode.telemetry.update();

        this.linearOpMode.telemetry.addData("Status ", "exited while loop");
        this.linearOpMode.telemetry.update();

        stopAllMotors();
    }

    public double getAngle() {
        robotOrientation = imu.getRobotYawPitchRollAngles();
        double gyroAngle = robotOrientation.getYaw(AngleUnit.DEGREES);
        linearOpMode.telemetry.addData("Gyro Angle", gyroAngle);
        linearOpMode.telemetry.update();
        return gyroAngle;
    }


    private boolean isTargetAngleReached(double currentAngle, double targetAngle) {
        final double ANGLE_TOLERANCE = 5; // degrees, adjust as needed
        return Math.abs(targetAngle - currentAngle) < ANGLE_TOLERANCE;
    }

    private void stopAllMotors() {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }


    public double getGyroAngle() {
        return robotOrientation.getYaw(AngleUnit.DEGREES);
    }

    /**
     * METHODS THAT MOVE FORWARD/BACKWARD USING MOTOR ENCODERS
     */
    public void forwardUsingEncoders (double inches, double speed){
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double targetPos = -(inches * ENCODER_TICKS_PER_INCH);

        // -3009 = 69 in
        // 1 inch = 43.6 ticks
        while (linearOpMode.opModeIsActive() && frontLeftMotor.getCurrentPosition() > targetPos) {
            frontLeftMotor.setPower(speed);
            backLeftMotor.setPower(speed);
            frontRightMotor.setPower(speed);
            backRightMotor.setPower(speed);
        }

        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);

        linearOpMode.telemetry.addData("Encoder pos front left", frontLeftMotor.getCurrentPosition());
        linearOpMode.telemetry.addData("Encoder pos back left", backLeftMotor.getCurrentPosition());
        linearOpMode.telemetry.addData("Encoder pos front right", frontRightMotor.getCurrentPosition());
        linearOpMode.telemetry.addData("Encoder pos back right", backRightMotor.getCurrentPosition());

        linearOpMode.telemetry.update();
    }

    public void backwardUsingEncoders (double inches, double speed){
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double targetPos = inches * ENCODER_TICKS_PER_INCH;

        while (linearOpMode.opModeIsActive() && frontLeftMotor.getCurrentPosition() < targetPos) {
            frontLeftMotor.setPower(-speed);
            backLeftMotor.setPower(-speed);
            frontRightMotor.setPower(-speed);
            backRightMotor.setPower(-speed);
        }

        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);

        linearOpMode.telemetry.addData("Encoder pos front left", frontLeftMotor.getCurrentPosition());
        linearOpMode.telemetry.addData("Encoder pos back left", backLeftMotor.getCurrentPosition());
        linearOpMode.telemetry.addData("Encoder pos front right", frontRightMotor.getCurrentPosition());
        linearOpMode.telemetry.addData("Encoder pos back right", backRightMotor.getCurrentPosition());

        linearOpMode.telemetry.update();
    }
    public void strafeRightUsingEncoders (double inches, double speed){
        // left wheels both moving outward in code
        // right wheels both moving inward in code

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double targetPos = -(inches * STRAFING_ENCODER_TICKS_PER_INCH);

        // -3009 = 69 in
        // 1 inch = 43.6 ticks
        while (linearOpMode.opModeIsActive() && frontLeftMotor.getCurrentPosition() > targetPos) {
            frontLeftMotor.setPower(speed);
            backLeftMotor.setPower(-speed);
            frontRightMotor.setPower(-speed);
            backRightMotor.setPower(speed);
        }

        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);

        linearOpMode.telemetry.addData("Encoder pos front left", frontLeftMotor.getCurrentPosition());
        linearOpMode.telemetry.addData("Encoder pos back left", backLeftMotor.getCurrentPosition());
        linearOpMode.telemetry.addData("Encoder pos front right", frontRightMotor.getCurrentPosition());
        linearOpMode.telemetry.addData("Encoder pos back right", backRightMotor.getCurrentPosition());

        linearOpMode.telemetry.update();
    }

    public void strafeLeftUsingEncoders (double inches, double speed){
        // left wheels both moving inward in code
        // right wheels both moving outward in code

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double targetPos = inches * STRAFING_ENCODER_TICKS_PER_INCH;

        // -3009 = 69 in
        // 1 inch = 43.6 ticks
        while (linearOpMode.opModeIsActive() && frontLeftMotor.getCurrentPosition() < targetPos) {
            frontLeftMotor.setPower(-speed);
            backLeftMotor.setPower(speed);
            frontRightMotor.setPower(speed);
            backRightMotor.setPower(-speed);
        }

        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);

        linearOpMode.telemetry.addData("Encoder pos front left", frontLeftMotor.getCurrentPosition());
        linearOpMode.telemetry.addData("Encoder pos back left", backLeftMotor.getCurrentPosition());
        linearOpMode.telemetry.addData("Encoder pos front right", frontRightMotor.getCurrentPosition());
        linearOpMode.telemetry.addData("Encoder pos back right", backRightMotor.getCurrentPosition());

        linearOpMode.telemetry.update();
    }

    /**
     * PID METHODS
     */


    public void pidMoveSliderToEncoderPosBrakeMode (int targetEncoderPos, double power, int slowDownEncoderPos) {
        isSliderMoving = true;

        getCurrentSliderEncoderPos();

        linearOpMode.telemetry.addLine(targetEncoderPos + "," + leftSliderMotor.getCurrentPosition());
        linearOpMode.telemetry.update();

        if (targetEncoderPos > leftSliderMotor.getCurrentPosition()) {
            pidSliderMoveUpBrakeMode(targetEncoderPos, power, slowDownEncoderPos);
        } else if (targetEncoderPos < leftSliderMotor.getCurrentPosition()) {
            pidSliderMoveDownBrakeMode(targetEncoderPos, power, slowDownEncoderPos);

        }

        isSliderMoving = false;
    }

    private void pidSliderMoveUpBrakeMode (int targetEncoderPos, double power, int slowDownEncoderPos) {
        this.leftSliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double encoderDiff;
        double kP = 0.01;

        getCurrentSliderEncoderPos();

        while ((getCurrentSliderEncoderPos() <= targetEncoderPos - slowDownEncoderPos) && linearOpMode.opModeIsActive()) {
            encoderDiff = leftSliderMotor.getCurrentPosition() - rightSliderMotor.getCurrentPosition();

            if (encoderDiff >= 0){
                leftSliderMotor.setPower((power - kP * encoderDiff));
                rightSliderMotor.setPower(power + kP * encoderDiff);
            } else {
                rightSliderMotor.setPower((power + kP * encoderDiff));
                leftSliderMotor.setPower(power - kP * encoderDiff);
            }
            linearOpMode.telemetry.addLine("Current Position: " + leftSliderMotor.getCurrentPosition());
            linearOpMode.telemetry.update();
        }


        while (getCurrentSliderEncoderPos() <= targetEncoderPos && linearOpMode.opModeIsActive()) {
            encoderDiff = leftSliderMotor.getCurrentPosition() - rightSliderMotor.getCurrentPosition();
            power = 0.3;

            if (encoderDiff >= 0) {
                leftSliderMotor.setPower(power - kP *encoderDiff);
                rightSliderMotor.setPower(power + kP * encoderDiff);
            } else {
                rightSliderMotor.setPower(power + kP *encoderDiff);
                leftSliderMotor.setPower(power-kP * encoderDiff);
            }
            linearOpMode.telemetry.addLine("Current Position: " + leftSliderMotor.getCurrentPosition());
            linearOpMode.telemetry.update();
        }


        holdSlider();
    }

    public int getCurrentSliderEncoderPos() {
        return (leftSliderMotor.getCurrentPosition() + rightSliderMotor.getCurrentPosition()) / 2;
    }

    public void holdSlider() {
        rightSliderMotor.setPower(0.05);
        leftSliderMotor.setPower(0.05);
    }

    private void pidSliderMoveDownBrakeMode (int targetEncoderPos, double power, int slowDownEncoderPos) {
        leftSliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double encoderDiff;
        double kP = 0.01;
        power = -power;

        while (getCurrentSliderEncoderPos() >= targetEncoderPos + slowDownEncoderPos && linearOpMode.opModeIsActive()) {
            encoderDiff = leftSliderMotor.getCurrentPosition() - rightSliderMotor.getCurrentPosition();

            if (encoderDiff >= 0) {
                leftSliderMotor.setPower(power - kP * encoderDiff);
                rightSliderMotor.setPower(power + kP * encoderDiff);
            } else {
                rightSliderMotor.setPower(power + kP * encoderDiff);
                leftSliderMotor.setPower(power - kP * encoderDiff);
            }
            linearOpMode.telemetry.addLine("Current Position: " + leftSliderMotor.getCurrentPosition());
            linearOpMode.telemetry.update();
        }

        while (getCurrentSliderEncoderPos() >= targetEncoderPos && linearOpMode.opModeIsActive() && !limitSwitch.isPressed()) {
            encoderDiff = leftSliderMotor.getCurrentPosition() - rightSliderMotor.getCurrentPosition();

            power = -0.2;

            if (encoderDiff >= 0) {
                leftSliderMotor.setPower(power - kP * encoderDiff);
                rightSliderMotor.setPower(power + kP * encoderDiff);
            } else {
                rightSliderMotor.setPower(power + kP * encoderDiff);
                leftSliderMotor.setPower(power - kP * encoderDiff);
            }
            linearOpMode.telemetry.addLine("Current Position: " + leftSliderMotor.getCurrentPosition());
            linearOpMode.telemetry.addData("", limitSwitch.isPressed());
            linearOpMode.telemetry.update();
        }

        linearOpMode.telemetry.addData("Status", "finished moving down");
        linearOpMode.telemetry.update();
        holdSlider();
    }

    public void resetSliderEncoderWithLimitSwitch() {
        while (!limitSwitch.isPressed()) {
            leftSliderMotor.setPower(-0.1);
            rightSliderMotor.setPower(-0.1);
        }

        if (limitSwitch.isPressed()) {
            linearOpMode.telemetry.addData("Status", "limit switch pressed");
            linearOpMode.telemetry.update();
            slidersResetByLimitSwitch = true;
        }

        leftSliderMotor.setPower(0);
        rightSliderMotor.setPower(0);

        leftSliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

}