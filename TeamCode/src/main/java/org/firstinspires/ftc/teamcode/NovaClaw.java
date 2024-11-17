package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp
public class NovaClaw extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        CRServo claw = hardwareMap.crservo.get("claw");
        boolean leftToggle = false;
        boolean rightToggle = false;

        DcMotor armMotor;
        armMotor = hardwareMap.dcMotor.get("armMotor");
        boolean armMotorToggleUp = false;
        boolean armMotorToggleDown = false;


        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // ARM MOVEMENT------------------------------------------------------------------------|
            if (gamepad2.x) {
                while(gamepad2.x) {
//                armMotorToggleUp = !armMotorToggleUp;
                    armMotor.setPower(0.6);
                }
                armMotor.setPower(0.0);
            }

            if (gamepad2.a) {
                while(gamepad2.a) {
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

            if (gamepad2.y){
                while(gamepad2.y) {
//                    leftToggle = !leftToggle;
                    claw.setPower(-1.0);
                }
                claw.setPower(0.0);
            }

            if (gamepad2.b){
                while (gamepad2.b) {
//                    rightToggle = !rightToggle;
                    claw.setPower(1.0);
                }
                claw.setPower(0.0);
            }

        }


    }
}
