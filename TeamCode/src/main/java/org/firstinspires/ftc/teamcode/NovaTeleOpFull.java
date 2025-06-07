package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;


// MAIN TELEOP

@TeleOp
public class NovaTeleOpFull extends LinearOpMode{

    public DcMotor leftSliderMotor;
    public DcMotor rightSliderMotor;

    public TouchSensor limitSwitch;
    public boolean isSliderMoving = false;
    public boolean slidersResetByLimitSwitch = false;

    @Override
    public void runOpMode() throws InterruptedException {

        CRServo claw = hardwareMap.crservo.get("claw");
        boolean leftToggle = false;
        boolean rightToggle = false;

        DcMotor armMotor;
        armMotor = hardwareMap.dcMotor.get("armMotor");
        boolean armMotorToggleUp = false;
        boolean armMotorToggleDown = false;

        CRServo pocket = hardwareMap.crservo.get("pocket");

        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        leftSliderMotor = hardwareMap.dcMotor.get("leftSliderMotor");
        rightSliderMotor = hardwareMap.dcMotor.get("rightSliderMotor");
//        leftSliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // These lines reset the encoder and do the job of the limit switch method
        leftSliderMotor.setPower(0);
        rightSliderMotor.setPower(0);
//        leftSliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightSliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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


            // ARM MOVEMENT------------------------------------------------------------------------|
            if (gamepad2.x) {
                while (gamepad2.x) {
//                armMotorToggleUp = !armMotorToggleUp;
                    armMotor.setPower(0.6);
                }
                armMotor.setPower(0.0);
            }

            if (gamepad2.a) {
                while (gamepad2.a) {
//                armMotorToggleDown = !armMotorToggleDown;
                    armMotor.setPower(-0.8);
                }
                armMotor.setPower(0.0);
            }


            // CLAW MOVEMENT-----------------------------------------------------------------------|
            telemetry.addLine();
            telemetry.addData("power:", claw.getPower());
            telemetry.update();

            // POWER SETTINGS
            // 0.5 - stationary     1.0 - max right     0.0 - max left

            if (gamepad2.y) {
                while (gamepad2.y) {
//                    leftToggle = !leftToggle;
                    claw.setPower(-1.0);
                }
                claw.setPower(0.0);
            }

            if (gamepad2.b) {
                while (gamepad2.b) {
//                    rightToggle = !rightToggle;
                    claw.setPower(1.0);
                }
                claw.setPower(0.0);
            }

            // POCKET MOVEMENT --------------------------------------------------------------------|
            if (gamepad2.left_bumper){ // up
                while (gamepad2.left_bumper) {
                    pocket.setPower(1.0); // right
                }
                pocket.setPower(0.0);
            }

            if (gamepad2.right_bumper){ // down
                while (gamepad2.right_bumper){
                    pocket.setPower(-1.0); // left
                }
                pocket.setPower(0.0);
            }

            // LINEAR SLIDES MOVEMENT -------------------------------------------------------------|
            telemetry.addData("right slider pos: ", rightSliderMotor.getCurrentPosition());
            telemetry.addData("left slider pos: ", leftSliderMotor.getCurrentPosition());
            telemetry.update();

            // If dpad left is pressed, sliders up to medium height
            if (gamepad2.dpad_left) {
//                pidMoveSliderToEncoderPosBrakeMode(1500, .1, 100);
//                if (Math.abs(rightSliderMotor.getCurrentPosition()) < 1500) {
                telemetry.addLine("left");
                telemetry.update();

//                if (Math.abs(getCurrentSliderEncoderPos()) < 1500) {
                if (Math.abs(rightSliderMotor.getCurrentPosition()) < 1000) {
                    moveUp(1500, 0.7, 10);
                } else {
                    moveDown(1500, 0.7, 10);
                }
            }

            // If dpad up is pressed, sliders up to high height
            if (gamepad2.dpad_up) {
                telemetry.addLine("up");
                telemetry.update();
//                pidMoveSliderToEncoderPosBrakeMode(500, .1, 100);
                if (Math.abs(rightSliderMotor.getCurrentPosition()) < 3500) {
//                if (Math.abs(getCurrentSliderEncoderPos()) < 1800) {
                    moveUp(3500, 0.7, 10);
                } else {
                    moveDown(3500, 0.7, 10);
                }
            }

            // If dpad down is pressed, sliders fully retract
            if (gamepad2.dpad_down) {
                telemetry.addData("down", true);
                telemetry.update();
//                pidMoveSliderToEncoderPosBrakeMode(0, .1, 100);
//                resetSliderEncoderWithLimitSwitch();
                moveDown(0, 0.6, -100);
            }

            // CLIMB MOVEMENT
            if (gamepad1.b){
                armMotor.setPower(-0.8);
                sleep(7000);
                armMotor.setPower(-0.7);
                sleep(1000);
                armMotor.setPower(-0.6);
                sleep(700);
                armMotor.setPower(-0.5);
                sleep(600);
                armMotor.setPower(-0.4);
                sleep(500);
                armMotor.setPower(-0.3);
                sleep(500);
                armMotor.setPower(-0.2);
                sleep(500);
                armMotor.setPower(-0.1);
                sleep(500);
                armMotor.setPower(0.0);
            }
        }
    }


    private void moveUp(int targetEncoderPos, double power, int slowDownEncoderPos){

        telemetry.addLine("in moveUp");
        telemetry.update();
        leftSliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // BRAKE - resist movement
        rightSliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (gamepad2.dpad_up){
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

        //        while ((Math.abs(rightSliderMotor.getCurrentPosition()) <= targetEncoderPos - slowDownEncoderPos) && opModeIsActive()) {
//            leftSliderMotor.setPower(power);
//            rightSliderMotor.setPower(power);
//        }

    }


    private void moveDown(int targetEncoderPos, double power, int slowDownEncoderPos){
        telemetry.addLine("in moveDown");
        telemetry.update();
        rightSliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftSliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (gamepad2.dpad_down){
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

        //
//        while ((Math.abs(rightSliderMotor.getCurrentPosition() ) >= targetEncoderPos - slowDownEncoderPos) && opModeIsActive()) {
//            telemetry.addData("current pos: ", getCurrentSliderEncoderPos());
//            telemetry.addData("pos: ", rightSliderMotor.getCurrentPosition());
//            telemetry.addData("target pos: ", targetEncoderPos - slowDownEncoderPos);
//            telemetry.update();
//            leftSliderMotor.setPower(power);
//            rightSliderMotor.setPower(power);
//        }

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
