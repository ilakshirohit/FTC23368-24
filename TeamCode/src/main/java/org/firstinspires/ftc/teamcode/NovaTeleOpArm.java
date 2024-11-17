    package org.firstinspires.ftc.teamcode;

    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.hardware.CRServo;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.Servo;
    //@TeleOp
    public class NovaTeleOpArm extends LinearOpMode {

        @Override
        public void runOpMode() throws InterruptedException {

            DcMotor armMotor;
            armMotor = hardwareMap.dcMotor.get("armMotor");
            boolean armPreviousButtonState = false;
            boolean armMotorToggleUp = false;
            boolean armMotorToggleDown = false;

            Servo claw;
            claw = hardwareMap.servo.get("claw");
            waitForStart();
            if (isStopRequested()) return;


            while (opModeIsActive()) {
                // ARM MOVEMENT -----------------------------------------------------------------------|
//                boolean currentButtonState = gamepad2.x;
    //        // continue to press so it goes where you want it to go?
    //        while (currentButtonState) {  // && !armPreviousButtonState
    //            armMotorToggleUp = !armMotorToggleUp;
    //            armMotor.setPower(armMotorToggleUp ? -0.5 : 0.5);
    //        }
    //            armPreviousButtonState = currentButtonState; // possibly useless
                if (gamepad2.x) {
                    armMotorToggleUp = !armMotorToggleUp;
                    armMotor.setPower(armMotorToggleUp ? 0.6 : 0);
                }

                if (gamepad2.a) {
                    armMotorToggleDown = !armMotorToggleDown;
                    armMotor.setPower(armMotorToggleDown ? -0.8 : 0);
                }

                // CLAW MOVEMENT-----------------------------------------------------------------------|
                telemetry.addLine();
                telemetry.addData("pos:", claw.getPosition());
                telemetry.update();
                telemetry.addData("pos:", claw.getPosition());
                telemetry.update();

                if (gamepad2.dpad_left){
                    while (gamepad2.dpad_left) {
                        telemetry.addLine("left");
                        telemetry.update();
                        double aim = claw.getPosition() - 0.5;
                        telemetry.addLine();
                        telemetry.addData("aimPos", aim);
                        telemetry.update();
                        claw.setPosition(aim);
                        sleep(30);
                    }
                }

                if (gamepad2.dpad_right){
                    while (gamepad2.dpad_right) {
                        telemetry.addLine("right");
                        telemetry.update();
                        double aim = claw.getPosition() + 0.5;
                        telemetry.addLine();
                        telemetry.addData("aimPos", aim);
                        telemetry.update();
                        claw.setPosition(aim);
                        sleep(30);
                    }
                }

                if (gamepad2.y) {
                    telemetry.addLine("y");
                    telemetry.update();
                    claw.setPosition(0.07); //prelim pos, edit after testing
                    sleep(20); // fix
                    claw.setPosition(0.02);
                }
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
