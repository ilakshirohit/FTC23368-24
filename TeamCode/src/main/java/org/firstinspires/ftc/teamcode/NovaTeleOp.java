package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


@TeleOp
public class NovaTeleOp extends LinearOpMode {

    public DcMotor leftSliderMotor;
    public DcMotor rightSliderMotor;
    public TouchSensor limitSwitch;
    public boolean isSliderMoving = false;
    public boolean slidersResetByLimitSwitch = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        DcMotor armMotor;
        armMotor = hardwareMap.dcMotor.get("armMotor");
        Servo claw;
        claw = hardwareMap.servo.get("claw");

        limitSwitch = hardwareMap.touchSensor.get("limitSwitch"); // ????

        boolean armPreviousButtonState = false;
        boolean armMotorToggle = false;
//        boolean clawPreviousButtonState = false;
//        boolean clawServoToggle = false;


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
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

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

            // ARM MOVEMENT -----------------------------------------------------------------------|
            boolean currentButtonState = gamepad2.x;
            //        // continue to press so it goes where you want it to go?
            //        while (currentButtonState) {  // && !armPreviousButtonState
            //            armMotorToggle = !armMotorToggle;
            //            armMotor.setPower(armMotorToggle ? -0.5 : 0.5);
            //        }
            //            armPreviousButtonState = currentButtonState; // possibly useless
            if (currentButtonState && !armPreviousButtonState) {
                armMotorToggle = !armMotorToggle;
                double power = 0.2;
                if (!armMotorToggle)
                    power = -1 * power;
                armMotor.setPower(power);
                sleep(100);
                armMotor.setPower(0);
            }

            if (gamepad1.dpad_up){
                while (gamepad1.dpad_up){
                    telemetry.addData("up", armMotor.getPower());
                    telemetry.update();
                    armMotor.setPower(0.2);
                }
                armMotor.setPower(0);
            }

            if (gamepad1.dpad_down){
                while (gamepad1.dpad_down){
                    telemetry.addData("down", armMotor.getPower());
                    telemetry.update();
                    armMotor.setPower(-0.2);
                }
                armMotor.setPower(0);
            }

            // CLAW MOVEMENT-----------------------------------------------------------------------|
            telemetry.addLine();
            telemetry.addData("pos:", claw.getPosition());
            telemetry.update();
            telemetry.addData("pos:", claw.getPosition());
            telemetry.update();

            if (gamepad2.y){
                while (gamepad2.y) {
                    telemetry.addLine("left");
                    telemetry.update();
                    double aim = claw.getPosition() - 0.001;
                    telemetry.addLine();
                    telemetry.addData("aimPos", aim);
                    telemetry.update();
                    claw.setPosition(aim);
                    sleep(30);
                }
            }

            if (gamepad2.a){
                while (gamepad2.a) {
                    telemetry.addLine("right");
                    telemetry.update();
                    double aim = claw.getPosition() + 0.001;
                    telemetry.addLine();
                    telemetry.addData("aimPos", aim);
                    telemetry.update();
                    claw.setPosition(aim);
                    sleep(30);
                }
            }

//            if (gamepad2.y) { // change button? do we need this?
//                telemetry.addLine("y");
//                telemetry.update();
//                claw.setPosition(0.07); //prelim pos, edit after testing
//                sleep(20); // fix
//                claw.setPosition(0.02);
//            }

            // LINEAR SLIDES MOVEMENT -------------------------------------------------------------|
            // If dpad left is pressed, slides up to medium height
            if (gamepad2.dpad_left) {
                pidMoveSliderToEncoderPosBrakeMode(1500, .4, 100);
            }

            // If dpad up is pressed, slides up to high height
            if (gamepad2.dpad_up) {
                pidMoveSliderToEncoderPosBrakeMode(1800, .4, 100);
            }

            // If dpad down is pressed, slides fully retract
            if (gamepad2.dpad_down) {
                pidMoveSliderToEncoderPosBrakeMode(0, .3, 100);
                resetSliderEncoderWithLimitSwitch();
            }
        }
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
