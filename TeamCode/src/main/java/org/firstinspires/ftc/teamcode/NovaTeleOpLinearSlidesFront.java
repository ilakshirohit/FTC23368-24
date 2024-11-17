package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;



public class NovaTeleOpLinearSlidesFront extends LinearOpMode {

    public DcMotor leftSliderMotor;
    public DcMotor rightSliderMotor;

    public TouchSensor limitSwitch;
    public boolean isSliderMoving = false;
    public boolean slidersResetByLimitSwitch = false;

    @Override
    public void runOpMode() throws InterruptedException {

//        DcMotor armMotor;
//        armMotor = hardwareMap.dcMotor.get("armMotor");
//        Servo claw;
//        claw = hardwareMap.servo.get("claw");

//        limitSwitch = hardwareMap.touchSensor.get("limitSwitch"); // ????

//        boolean armPreviousButtonState = false;
//        boolean armMotorToggle = false;
//        boolean clawPreviousButtonState = false;
//        boolean clawServoToggle = false;

        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

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



        // Reverse the right side motors. THIS IS WRONG FOR OUR SETUP!!!!!
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // DRIVEBASE MOVEMENT -----------------------------------------------------------------|
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            telemetry.addLine("Current Positions: X: " + x + "; Y: " + y + "; RX: " + rx);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            telemetry.addLine("Denominator: " + denominator);
            telemetry.addLine("FL, FR, BL, BR Power: " + frontLeftPower + "," + frontRightPower
                    + "," + backLeftPower + "," + backRightPower);
            telemetry.update();

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // NO ARM OR CLAW MOVEMENT ON RIGHT NOW

//            // ARM MOVEMENT -----------------------------------------------------------------------|
//            // up and down movement, so it needs to stop or it will go straight into slides/ground
//            boolean currentButtonState = gamepad2.x;
//
//            while (currentButtonState) {
//                armMotorToggle = !armMotorToggle;
//                armMotor.setPower(armMotorToggle ? -0.5 : 0.5);
//            }
//
////            armPreviousButtonState = currentButtonState; // possibly useless
//
//            // CLAW MOVEMENT-----------------------------------------------------------------------|
//            if (gamepad2.b){
//                claw.setPosition(0.25); //prelim pos, edit after testing
//                sleep(2000); // fix
//                claw.setPosition(0);
//            }
//
////            boolean clawCurrentButtonState = gamepad2.b;
////
////            if (clawCurrentButtonState && !clawPreviousButtonState){
////                clawServoToggle = !clawServoToggle;
////
////                claw.setPosition(0)
////            }

            // LINEAR SLIDES MOVEMENT -------------------------------------------------------------|
            // If dpad left is pressed, sliders up to medium height
            if (gamepad2.dpad_left) {
//                pidMoveSliderToEncoderPosBrakeMode(1500, .1, 100);
//                if (Math.abs(rightSliderMotor.getCurrentPosition()) < 1500) {
                telemetry.addLine("left");
                telemetry.update();

                if (Math.abs(getCurrentSliderEncoderPos()) < 1500) {
                    moveUp(1500, 0.4, 10);
                } else {
                    moveDown(1500, 0.4, 10);
                }
            }

            // If dpad up is pressed, sliders up to high height
            if (gamepad2.dpad_up) {
                telemetry.addLine("up");
                telemetry.update();
//                pidMoveSliderToEncoderPosBrakeMode(500, .1, 100);
//                if (Math.abs(rightSliderMotor.getCurrentPosition()) < 1800) {
                if (Math.abs(getCurrentSliderEncoderPos()) < 1800) {
                    moveUp(1800, 0.4, 10);
                } else {
                    moveDown(1800, 0.4, 10);
                }
            }

            // If dpad down is pressed, sliders fully retract
            if (gamepad2.dpad_down) {
                telemetry.addData("down", true);
                telemetry.update();
//                pidMoveSliderToEncoderPosBrakeMode(0, .1, 100);
//                resetSliderEncoderWithLimitSwitch();
                moveDown(0, 0.3, -10);
            }
        }
    }


    private void moveUp(int targetEncoderPos, double power, int slowDownEncoderPos){

        telemetry.addLine("in moveUp");
        telemetry.update();
        leftSliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // BRAKE - resist movement
        rightSliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        leftSliderMotor.setDirection(DcMotorSimple.Direction.REVERSE); going wrong way now

        while ((Math.abs(leftSliderMotor.getCurrentPosition()) <= targetEncoderPos - slowDownEncoderPos) && opModeIsActive()) {
            telemetry.addData("current pos: ", getCurrentSliderEncoderPos());
            telemetry.addData("right pos: ", rightSliderMotor.getCurrentPosition());
            telemetry.addData("left pos: ", leftSliderMotor.getCurrentPosition());
            telemetry.addData("target pos: ", targetEncoderPos - slowDownEncoderPos);
            telemetry.update();
            leftSliderMotor.setPower(power);
            rightSliderMotor.setPower(power);
        }
        leftSliderMotor.setPower(0);
        rightSliderMotor.setPower(0);
    }


    private void moveDown(int targetEncoderPos, double power, int slowDownEncoderPos){
        telemetry.addLine("in moveDown");
        telemetry.update();
        rightSliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftSliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while ((Math.abs(leftSliderMotor.getCurrentPosition() ) >= targetEncoderPos - slowDownEncoderPos) && opModeIsActive()) {
            telemetry.addData("current pos: ", getCurrentSliderEncoderPos());
            telemetry.addData("pos: ", rightSliderMotor.getCurrentPosition());
            telemetry.addData("target pos: ", targetEncoderPos - slowDownEncoderPos);
            telemetry.update();
            leftSliderMotor.setPower(power);
            rightSliderMotor.setPower(power);
        }
        leftSliderMotor.setPower(0);
        rightSliderMotor.setPower(0);
        leftSliderMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightSliderMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    // PID METHODS --------------------------------------------------------------------------------|

    public void pidMoveSliderToEncoderPosBrakeMode (int targetEncoderPos, double power, int slowDownEncoderPos) {
        isSliderMoving = true;

        getCurrentSliderEncoderPos();

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

        while ((getCurrentSliderEncoderPos() <= targetEncoderPos - slowDownEncoderPos) && opModeIsActive()) {
            encoderDiff = leftSliderMotor.getCurrentPosition() - rightSliderMotor.getCurrentPosition();

            if (encoderDiff >= 0){
                leftSliderMotor.setPower((power - kP * encoderDiff));
                rightSliderMotor.setPower(power + kP * encoderDiff);
            } else {
                rightSliderMotor.setPower((power + kP * encoderDiff));
                leftSliderMotor.setPower(power - kP * encoderDiff);
            }
        }


        while (getCurrentSliderEncoderPos() <= targetEncoderPos && opModeIsActive()) {
            encoderDiff = leftSliderMotor.getCurrentPosition() - rightSliderMotor.getCurrentPosition();
            power = 0.3;

            if (encoderDiff >= 0) {
                leftSliderMotor.setPower(power - kP *encoderDiff);
                rightSliderMotor.setPower(power + kP * encoderDiff);
            } else {
                rightSliderMotor.setPower(power + kP *encoderDiff);
                leftSliderMotor.setPower(power-kP * encoderDiff);
            }
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

        while (getCurrentSliderEncoderPos() >= targetEncoderPos + slowDownEncoderPos && opModeIsActive()) {
            encoderDiff = leftSliderMotor.getCurrentPosition() - rightSliderMotor.getCurrentPosition();

            if (encoderDiff >= 0) {
                leftSliderMotor.setPower(power - kP * encoderDiff);
                rightSliderMotor.setPower(power + kP * encoderDiff);
            } else {
                rightSliderMotor.setPower(power + kP * encoderDiff);
                leftSliderMotor.setPower(power - kP * encoderDiff);
            }
        }

        while (getCurrentSliderEncoderPos() >= targetEncoderPos && opModeIsActive()) {
            encoderDiff = leftSliderMotor.getCurrentPosition() - rightSliderMotor.getCurrentPosition();

            power = -0.1;

            if (encoderDiff >= 0) {
                leftSliderMotor.setPower(power - kP * encoderDiff);
                rightSliderMotor.setPower(power + kP * encoderDiff);
            } else {
                rightSliderMotor.setPower(power + kP * encoderDiff);
                leftSliderMotor.setPower(power - kP * encoderDiff);
            }
        }

        holdSlider();
    }

    public void resetSliderEncoderWithLimitSwitch() {
        while (!limitSwitch.isPressed()) {
            leftSliderMotor.setPower(-0.1);
            rightSliderMotor.setPower(-0.1);
        }

        if (limitSwitch.isPressed()) {
            slidersResetByLimitSwitch = true;
        }

        leftSliderMotor.setPower(0);
        rightSliderMotor.setPower(0);

        leftSliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}