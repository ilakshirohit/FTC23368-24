package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;



// THIS CLASS IS INVALID
// it gets to the ascent zone from the right side (which is the wrong side) and you also don't get points for doing this!

//@Autonomous
public class NovaAutoBlueRight extends LinearOpMode {

    public DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status: ", "Robot Initialized");
        telemetry.update();

        waitForStart();

        frontLeftMotor.setPower(0.3);
        backLeftMotor.setPower(0.3);
        frontRightMotor.setPower(0.3);
        backRightMotor.setPower(0.3);
        sleep(1500);
        frontRightMotor.setPower(0.5);
        frontLeftMotor.setPower(-0.5);
        backLeftMotor.setPower(0.5);
        backRightMotor.setPower(-0.5);
        sleep(500);
        frontLeftMotor.setPower(0.3);
        backLeftMotor.setPower(0.3);
        frontRightMotor.setPower(0.3);
        backRightMotor.setPower(0.3);
        sleep(2500);
        frontRightMotor.setPower(-0.5);
        frontLeftMotor.setPower(0.5);
        backLeftMotor.setPower(-0.5);
        backRightMotor.setPower(0.5);
        sleep(1800);
        frontLeftMotor.setPower(0.3);
        backLeftMotor.setPower(0.3);
        frontRightMotor.setPower(0.3);
        backRightMotor.setPower(0.3);
        sleep(3000);
        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

    }
}
