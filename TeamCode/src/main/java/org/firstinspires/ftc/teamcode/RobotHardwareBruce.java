package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class RobotHardwareBruce {

    // Declare OpMode members
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    public DcMotor fl = null;
    public DcMotor fr = null;
    public DcMotor bl = null;
    public DcMotor br = null;
    public DcMotor slideMotor = null;


    public DcMotor arm = null;
    private double motorArmZeroPower = 0.0;
    private double motorArmPower = 1.0;
    private int motorArmPositionOne = 10;
    private int motorArmPositionTwo = 90;


    public Servo leftclawServo = null;
    public Servo rightclawServo = null;
    public Servo reachServo = null;


    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardwareBruce(LinearOpMode opMode) {
        myOpMode = opMode;
    }

    // Initialize all the robot's hardware
    public void init()    {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        fl  = myOpMode.hardwareMap.get(DcMotor.class, "FL"); //Front Left
        fr  = myOpMode.hardwareMap.get(DcMotor.class, "FR"); //Front Right
        bl = myOpMode.hardwareMap.get(DcMotor.class, "BL"); //Back Left
        br = myOpMode.hardwareMap.get(DcMotor.class, "BR"); //Back Right

        //Set drive motor directions
        fl.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);


        leftclawServo = myOpMode.hardwareMap.get(Servo.class, "leftClaw");
        rightclawServo = myOpMode.hardwareMap.get(Servo.class, "rightClaw");
        reachServo = myOpMode.hardwareMap.get(Servo.class, "reach");
        slideMotor = myOpMode.hardwareMap.get(DcMotor.class, "slide");



        //initMotorArm();



        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();

    }

    public void initMotorArm(){
        arm = myOpMode.hardwareMap.get(DcMotor.class, "arm"); //Arm
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setPower(motorArmZeroPower);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runMotorArmToPosition(int position){
        arm.setTargetPosition(position);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(motorArmPower);
        while(arm.isBusy()){
            myOpMode.telemetry.addData("Arm is busy", ".2f", "");
        }
        //arm.setPower(motorArmZeroPower);
    }

    public void abc(int position){
        while(arm.getCurrentPosition() != position){
            while(arm.getCurrentPosition() > position){
                arm.setPower(-0.5);
            }
            while(arm.getCurrentPosition() < position){
                arm.setPower(0.5);
            }
        }
    }

    public void driveRobot(double axial, double lateral, double yaw, double speed) {

        // Calculate wheel speeds and set their power
        double flPower = (axial + lateral + yaw) * speed;
        double frPower = (axial - lateral - yaw) * speed;
        double blPower = (axial - lateral + yaw) * speed;
        double brPower = (axial + lateral - yaw) * speed;
        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);
    }


    public void slideRobot(double slidePower){
        slideMotor.setPower(slidePower);
    }

    public void armRobot(){

    }

    public void closeClaw() {
        leftclawServo.setPosition(0.4);
        rightclawServo.setPosition(0.6);
        //Wait here for a second (somehow)
        leftclawServo.setPosition(leftclawServo.getPosition());
        rightclawServo.setPosition(rightclawServo.getPosition());
    }

    public void openClaw() {
        leftclawServo.setPosition(0.5);
        rightclawServo.setPosition(0.5);
    }
}