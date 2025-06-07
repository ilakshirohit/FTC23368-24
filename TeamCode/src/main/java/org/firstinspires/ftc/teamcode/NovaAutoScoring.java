package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Autonomous
public class NovaAutoScoring extends LinearOpMode{
    public DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    public DcMotor leftSliderMotor, rightSliderMotor;
    public CRServo claw;
    public DcMotor armMotor;
    public CRServo pocket;

    @Override
    public void runOpMode() throws InterruptedException {

        // set up
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        leftSliderMotor = hardwareMap.dcMotor.get("leftSliderMotor");
        rightSliderMotor = hardwareMap.dcMotor.get("rightSliderMotor");

        claw = hardwareMap.crservo.get("claw");

        armMotor = hardwareMap.dcMotor.get("armMotor");

        pocket = hardwareMap.crservo.get("pocket");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

//        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftSliderMotor.setPower(0);
        rightSliderMotor.setPower(0);
        leftSliderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSliderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status: ", "Robot Initialized");
        telemetry.update();


        waitForStart();


        // run operations

        // in case we have to delay for our partners
        int time = 0;
        sleep(time);

        // strafe away from wall
        frontLeftMotor.setPower(-0.3);
        backLeftMotor.setPower(0.3);
        frontRightMotor.setPower(0.3);
        backRightMotor.setPower(-0.3);
        sleep(800);
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);

        // take preloaded sample to net zone
        telemetry.addData("Status: ", "Taking sample to low basket");
        telemetry.update();
        frontLeftMotor.setPower(-0.4);
        backLeftMotor.setPower(-0.4);
        frontRightMotor.setPower(-0.4);
        backRightMotor.setPower(-0.4);
        sleep(550);
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);

        frontLeftMotor.setPower(-0.3);
        backLeftMotor.setPower(-0.3);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
        sleep(1600);
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);

        frontLeftMotor.setPower(-0.1);
        backLeftMotor.setPower(-0.1);
        frontRightMotor.setPower(-0.1);
        backRightMotor.setPower(-0.1);
        sleep(350);
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);

        // drop in low basket
        telemetry.addData("Status: ", "Dropping sample in low basket");
        telemetry.update();
        moveTimeBased(2100, 0.7);
        leftSliderMotor.setPower(0.1);
        rightSliderMotor.setPower(0.1);
        pocket.setPower(-1.0);
        sleep(1200);
        pocket.setPower(0);
        pocket.setPower(1.0);
        sleep(1200);
        pocket.setPower(0);
        moveTimeBased(2100, -0.7);

//        // strafe right
//        frontLeftMotor.setPower(0.5);
//        backLeftMotor.setPower(-0.5);
//        frontRightMotor.setPower(-0.5);
//        backRightMotor.setPower(0.5);
//        sleep(1000);
//
//        // rotate right backwards
//        frontLeftMotor.setPower(0.1); // possibly make left power 0
//        backLeftMotor.setPower(0.1); // possibly make left power 0
//        frontRightMotor.setPower(-0.5);
//        backRightMotor.setPower(-0.5);
//        sleep(700);
//        // go forward towards

        frontLeftMotor.setPower(0.3);
        backLeftMotor.setPower(0.3);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
        sleep(1200);
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);

//        frontLeftMotor.setPower(0);
//        backLeftMotor.setPower(0);
//        frontRightMotor.setPower(0.3);
//        backRightMotor.setPower(0.3);
//        sleep(200);
//        frontLeftMotor.setPower(0);
//        backLeftMotor.setPower(0);
//        frontRightMotor.setPower(0);
//        backRightMotor.setPower(0);

        // go back to park
        telemetry.addData("Status: ", "Parking");
        telemetry.update();
        frontLeftMotor.setPower(0.5);
        backLeftMotor.setPower(0.5);
        frontRightMotor.setPower(0.5);
        backRightMotor.setPower(0.5);
        sleep(3500);
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    private void moveTimeBased(int time, double power){
        telemetry.addLine("in move");
        telemetry.update();
//        rightSliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        leftSliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftSliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//            telemetry.addData("current pos: ", getCurrentSliderEncoderPos());
        telemetry.addData("pos: ", rightSliderMotor.getCurrentPosition());
        telemetry.addData("set time: ", time);
        telemetry.update();

        leftSliderMotor.setPower(power);
        rightSliderMotor.setPower(power);

        sleep(time);

        leftSliderMotor.setPower(0);
        rightSliderMotor.setPower(0);

        leftSliderMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightSliderMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    private void moveUp(int targetEncoderPos, double power, int slowDownEncoderPos){

        telemetry.addLine("in moveUp");
        telemetry.update();
        leftSliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // BRAKE - resist movement
        rightSliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        leftSliderMotor.setDirection(DcMotorSimple.Direction.REVERSE); going wrong way now

        while ((Math.abs(leftSliderMotor.getCurrentPosition()) <= targetEncoderPos - slowDownEncoderPos) && opModeIsActive()) {
//            telemetry.addData("current pos: ", getCurrentSliderEncoderPos());
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
//            telemetry.addData("current pos: ", getCurrentSliderEncoderPos());
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


}
