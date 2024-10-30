    package org.firstinspires.ftc.teamcode;

    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.Servo;

    @TeleOp
    public class NovaTeleOpArm extends LinearOpMode {

        @Override
        public void runOpMode() throws InterruptedException {

            DcMotor armMotor;
            armMotor = hardwareMap.dcMotor.get("armMotor");
            boolean armPreviousButtonState = false;
            boolean armMotorToggle = false;

            Servo claw;
            claw = hardwareMap.servo.get("claw");
            waitForStart();
            if (isStopRequested()) return;


            while (opModeIsActive()) {
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
                    double power = 0.8;
                    if (!armMotorToggle)
                        power = -1 * power;
                    armMotor.setPower(power);
                    sleep(200);
                    armMotor.setPower(0);
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
                        double aim = claw.getPosition() - 0.001;
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
                        double aim = claw.getPosition() + 0.001;
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
