package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class NovaArm extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor armMotor;
        armMotor = hardwareMap.dcMotor.get("armMotor");
        boolean armPreviousButtonState = false;
        boolean armMotorToggle = false;

        Servo claw;
        claw = hardwareMap.servo.get("claw");

        // ARM MOVEMENT -----------------------------------------------------------------------|
        boolean currentButtonState = gamepad2.x;

//        // continue to press so it goes where you want it to go?
//        while (currentButtonState) {  // && !armPreviousButtonState
//            armMotorToggle = !armMotorToggle;
//            armMotor.setPower(armMotorToggle ? -0.5 : 0.5);
//        }
//            armPreviousButtonState = currentButtonState; // possibly useless

        if (currentButtonState && !armPreviousButtonState){
            armMotorToggle = !armMotorToggle;
            double power = 0.5;
            if (!armMotorToggle)
                power = -1*power;
            armMotor.setPower(power);
            wait(700);
            armMotor.setPower(0);
        }

        // CLAW MOVEMENT-----------------------------------------------------------------------|
        if (gamepad2.b){
            claw.setPosition(0.25); //prelim pos, edit after testing
            sleep(2000); // fix
            claw.setPosition(0);
        }

//            boolean clawCurrentButtonState = gamepad2.b;
//
//            if (clawCurrentButtonState && !clawPreviousButtonState){
//                clawServoToggle = !clawServoToggle;
//
//                claw.setPosition(0)
//            }

    }
}
