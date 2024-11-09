package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOp")
public class GoodTEleOp extends LinearOpMode {
    Hardware robot;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot = new Hardware(this);
        robot.init();
        int claw = 0;

        waitForStart();

        while (opModeIsActive()){
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
            double slidePower = -gamepad2.left_stick_y;
            double speed = (gamepad1.right_trigger/.8)+.2;
            double armPower = gamepad2.right_stick_y;
            boolean clawOpen = gamepad2.right_trigger<.25;
            boolean clawMiddle = gamepad2.right_trigger>=.25 && gamepad2.right_trigger<.75;
            boolean clawClose = gamepad2.right_trigger>=.75;

            robot.armMotor.setPower(armPower*.75);
            /*
            if (armPower >=.1 || armPower <=-.1) {
                if (robot.armMotor.getMode()!= DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
                    robot.armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
                robot.armMotor.setPower(armPower);
            } else {
                if (robot.armMotor.getMode()!= DcMotor.RunMode.RUN_TO_POSITION) {
                    robot.armMotor.setTargetPosition(robot.armMotor.getCurrentPosition());
                    robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.armMotor.setPower(.6);
                }
            }*/

            if(clawOpen){ //open
                claw = 0;
            }
            if(clawMiddle){
                claw = 1;
            }
            if(clawClose){
                claw = 2;
            }

            robot.drive(axial, lateral, yaw, speed);
            robot.arm(slidePower, claw, armPower);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Arm", robot.armMotor.getCurrentPosition());
            telemetry.update();

        }
    }

}
