package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
public class NovaPocket extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        CRServo pocket = hardwareMap.crservo.get("pocket");

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()){
            if (gamepad2.left_bumper){ // up
                while (gamepad2.left_bumper) {
                    pocket.setPower(0.75); // right
                }
                pocket.setPower(0.0);
            }

            if (gamepad2.right_bumper){ // down
                while (gamepad2.right_bumper){
                    pocket.setPower(-0.75); // left
                }
                pocket.setPower(0.0);
            }

        }

    }
}
