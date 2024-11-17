package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;


@TeleOp
public class NovaLinearSlide extends LinearOpMode {
    public DcMotor leftSliderMotor;
    public DcMotor rightSliderMotor;

    public TouchSensor limitSwitch;
    public boolean isSliderMoving = false;
    public boolean slidersResetByLimitSwitch = false;

    @Override
    public void runOpMode() throws InterruptedException {

        leftSliderMotor = hardwareMap.dcMotor.get("leftSliderMotor");
        rightSliderMotor = hardwareMap.dcMotor.get("rightSliderMotor");
//        leftSliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // These lines reset the encoder and do the job of the limit switch method
        leftSliderMotor.setPower(0);
        rightSliderMotor.setPower(0);

        leftSliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSliderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSliderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            telemetry.addData("left pos: ", leftSliderMotor.getCurrentPosition());
            telemetry.addData("right pos: ", rightSliderMotor.getCurrentPosition());

            telemetry.update();
            // LINEAR SLIDES MOVEMENT -------------------------------------------------------------|
            // If dpad left is pressed, sliders up to medium height 1500

            if (gamepad2.a){
                telemetry.addLine("pressed");
                telemetry.update();
            }

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

            // If dpad up is pressed, sliders up to high height 1800
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

        telemetry.addLine("end");
        telemetry.update();

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



//    private void moveUp(int targetEncoderPos, double power, int slowDownEncoderPos){
//
//        leftSliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightSliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        while ((Math.abs(rightSliderMotor.getCurrentPosition()) <= targetEncoderPos - slowDownEncoderPos) && opModeIsActive()) {
//            telemetry.addData("current pos: ", getCurrentSliderEncoderPos());
//            telemetry.addData("pos: ", rightSliderMotor.getCurrentPosition());
//            telemetry.addData("target pos: ", targetEncoderPos - slowDownEncoderPos);
//            telemetry.update();
//            leftSliderMotor.setPower(power);
//            rightSliderMotor.setPower(power);
//        }
//        leftSliderMotor.setPower(0);
//        rightSliderMotor.setPower(0);
//    }

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

        if (targetEncoderPos > rightSliderMotor.getCurrentPosition()) {
            pidSliderMoveUpBrakeMode(targetEncoderPos, power, slowDownEncoderPos);
        } else if (targetEncoderPos < rightSliderMotor.getCurrentPosition()) {
            pidSliderMoveDownBrakeMode(targetEncoderPos, power, slowDownEncoderPos);
        }

        isSliderMoving = false;
    }

    private void pidSliderMoveUpBrakeMode (int targetEncoderPos, double power, int slowDownEncoderPos) {
        this.leftSliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double encoderDiff;
        double kP = 0;

        getCurrentSliderEncoderPos();

        while ((getCurrentSliderEncoderPos() <= targetEncoderPos - slowDownEncoderPos) && opModeIsActive()) {
            telemetry.addLine("in while loop 1");

            encoderDiff = leftSliderMotor.getCurrentPosition() - rightSliderMotor.getCurrentPosition();

            telemetry.addData("left pos: ", leftSliderMotor.getCurrentPosition());
            telemetry.addData("right pos: ", rightSliderMotor.getCurrentPosition());
            telemetry.addData("current pos: ", getCurrentSliderEncoderPos());
            telemetry.addData("target: ", targetEncoderPos);
            telemetry.addData("encoder diff: ", encoderDiff);
            telemetry.addData("current power: ", rightSliderMotor.getPower());
            telemetry.addData("set power: ", power);
            telemetry.update();
            sleep(2000);

            if (encoderDiff >= 0){
                leftSliderMotor.setPower((power - kP * encoderDiff));
                telemetry.addData("power: ", power + kP * encoderDiff);
                telemetry.update();
                rightSliderMotor.setPower(power + kP * encoderDiff);
            } else {
                telemetry.addData("power: ", power + kP * encoderDiff);
                telemetry.update();
                rightSliderMotor.setPower((power + kP * encoderDiff));
                leftSliderMotor.setPower(power - kP * encoderDiff);
            }
        }


        while (getCurrentSliderEncoderPos() <= targetEncoderPos && opModeIsActive()) {
            telemetry.addLine("in while loop 2");
            telemetry.update();
            encoderDiff = leftSliderMotor.getCurrentPosition() - rightSliderMotor.getCurrentPosition();
            power = 0.3;

            if (encoderDiff >= 0) {
                leftSliderMotor.setPower(power - kP *encoderDiff);
                telemetry.addData("power: ", power + kP * encoderDiff);
                telemetry.update();

                rightSliderMotor.setPower(power + kP * encoderDiff);
            } else {
                telemetry.addData("power: ", power + kP * encoderDiff);
                telemetry.update();

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
        double kP = 0.0;
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
