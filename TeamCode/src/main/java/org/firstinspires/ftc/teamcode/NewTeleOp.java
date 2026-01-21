package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "DrivingOPMODE3 (Blocks to Java)")
public class DrivingOPMODE3 extends LinearOpMode {

    private DcMotor BR;
    private DcMotor BL;
    private DcMotor FR;
    private DcMotor FL;
    private DcMotor Artifactlauncher;
    private DcMotor intakestring;
    private DcMotor intakewheels;
    private Servo blocker;
    private Servo transfer;
    private Servo angle;

    /**
     * This OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
     * This code will work with either a Mecanum-Drive or an X-Drive train.
     * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
     *
     * Also note that it is critical to set the correct rotation direction for each motor. See details below.
     *
     * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
     * Each motion axis is controlled by one Joystick axis.
     *
     * 1) Axial -- Driving forward and backward -- Left-joystick Forward/Backward
     * 2) Lateral -- Strafing right and left -- Left-joystick Right and Left
     * 3) Yaw -- Rotating Clockwise and counter clockwise -- Right-joystick Right and Left
     *
     * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
     * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
     * the direction of all 4 motors (see code below).
     */
    @Override
    public void runOpMode() {
        ElapsedTime runtime;
        float axial;
        float lateral;
        float yaw;
        float leftFrontPower;
        float rightFrontPower;
        float leftBackPower;
        float rightBackPower;
        double max;
        double Power;

        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        Artifactlauncher = hardwareMap.get(DcMotor.class, "Artifact launcher");
        intakestring = hardwareMap.get(DcMotor.class, "intake string");
        intakewheels = hardwareMap.get(DcMotor.class, "intake wheels");
        blocker = hardwareMap.get(Servo.class, "blocker");
        transfer = hardwareMap.get(Servo.class, "transfer");
        angle = hardwareMap.get(Servo.class, "angle");

        waitForStart();
        while (opModeIsActive()) {
            // This is the competition opmode for teleOp two player driving
            // This is a gamepad 1 and 2 setup
            // This is where we initialize but we don't initialize.
            // Do not touch. It has the right programming.
            runtime = new ElapsedTime();
            // Telemetry tells us motor inputs
            // This shows the direction the motors are set to run to.
            BR.setDirection(DcMotor.Direction.REVERSE);
            BL.setDirection(DcMotor.Direction.REVERSE);
            FR.setDirection(DcMotor.Direction.FORWARD);
            FL.setDirection(DcMotor.Direction.FORWARD);
            // This is what stick is foward and backward, left and right, and strafing right and left
            axial = -gamepad1.left_stick_y;
            lateral = gamepad1.right_stick_x;
            yaw = gamepad1.left_stick_x;
            // This is how we calculate the direction of each wheel
            leftFrontPower = (axial - lateral) + yaw;
            rightFrontPower = (axial - lateral) - yaw;
            leftBackPower = axial + lateral + yaw;
            rightBackPower = (axial + lateral) - yaw;
            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(leftFrontPower), Math.abs(rightFrontPower), Math.abs(leftBackPower), Math.abs(rightBackPower)));
            // This is to stop it from overloading or going kaboom!
            if (max > 1) {
                leftFrontPower = (float) (leftFrontPower / max);
                rightFrontPower = (float) (rightFrontPower / max);
                leftBackPower = (float) (leftBackPower / max);
                rightBackPower = (float) (rightBackPower / max);
            }
            // This shows how the power is set to regular and turbo
            if (gamepad1.right_trigger > 0) {
                Power = 1;
            } else {
                Power = 0.5;
            }
            // Send calculated power to wheels.
            // This is the power for the wheels
            BR.setPower(rightBackPower * Power);
            FR.setPower(rightFrontPower * Power);
            BL.setPower(leftBackPower * Power);
            FL.setPower(leftFrontPower * Power);
            // Show the elapsed game time and wheel power.
            // This is the artifact launcher code to launch an artifact
            if (gamepad2.left_trigger > 0) {
                Artifactlauncher.setPower(0.76);
            } else if (gamepad2.right_trigger > 0) {
                Artifactlauncher.setPower(0.91);
            } else {
                Artifactlauncher.setPower(0);
            }
            // X sets the intake to full and B sets it to zero
            if (gamepad2.right_stick_y < 0) {
                intakestring.setDirection(DcMotor.Direction.REVERSE);
                intakestring.setPower(1);
            } else {
                intakestring.setPower(0);
            }
            if (gamepad2.left_stick_y < 0) {
                intakewheels.setDirection(DcMotor.Direction.REVERSE);
                intakewheels.setPower(1);
            } else if (gamepad2.left_stick_y > 0) {
                intakewheels.setDirection(DcMotor.Direction.FORWARD);
                intakewheels.setPower(1);
            } else {
                intakewheels.setPower(0);
            }
            if (gamepad2.y) {
                blocker.setPosition(0.87);
            }
            // If you push y on gamepad 2 you will push the artifacts into the launcher and put the blocker up
            if (gamepad2.y) {
                transfer.setPosition(0.55);
            } else {
                blocker.setPosition(0.75);
                transfer.setPosition(0.22);
            }
            if (gamepad2.left_bumper) {
                angle.setPosition(0.75);
            }
            if (gamepad2.right_bumper) {
                angle.setPosition(0.7);
        }
    }
}