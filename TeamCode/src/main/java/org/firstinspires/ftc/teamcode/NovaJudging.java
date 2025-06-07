package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class NovaJudging extends LinearOpMode{

    public DcMotor leftSliderMotor;
    public DcMotor rightSliderMotor;


    @Override
    public void runOpMode() throws InterruptedException {

        CRServo claw = hardwareMap.crservo.get("claw");

        DcMotor armMotor;
        armMotor = hardwareMap.dcMotor.get("armMotor");

        CRServo pocket = hardwareMap.crservo.get("pocket");

        leftSliderMotor = hardwareMap.dcMotor.get("leftSliderMotor");
        rightSliderMotor = hardwareMap.dcMotor.get("rightSliderMotor");

        // These lines reset the encoder and do the job of the limit switch method
        leftSliderMotor.setPower(0);
        rightSliderMotor.setPower(0);
        leftSliderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSliderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
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
                telemetry.addLine("left");
                telemetry.update();

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
                if (Math.abs(rightSliderMotor.getCurrentPosition()) < 3500) {
                    moveUp(3500, 0.7, 10);
                } else {
                    moveDown(3500, 0.7, 10);
                }
            }

            // If dpad down is pressed, sliders fully retract
            if (gamepad2.dpad_down) {
                telemetry.addData("down", true);
                telemetry.update();
                moveDown(0, 0.6, -100);
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

    }

    // PID METHODS --------------------------------------------------------------------------------|
    // took out movement ones for this
    public int getCurrentSliderEncoderPos() {
        return (leftSliderMotor.getCurrentPosition() + rightSliderMotor.getCurrentPosition()) / 2;
    }

}
