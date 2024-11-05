package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


public class RobotHardwareBruce {

    // Declare OpMode members
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    public DcMotor fl = null;
    public DcMotor fr = null;
    public DcMotor bl = null;
    public DcMotor br = null;
    public DcMotor slides = null;

    public Servo elbowServo = null;
    public Servo leftclawServo = null;
    public Servo rightclawServo = null;

    public enum directions{
        FORWARD,
        BACKWARD,
        RIGHT,
        LEFT
    }


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

        slides = myOpMode.hardwareMap.get(DcMotor.class, "slide"); //Slide
        elbowServo = myOpMode.hardwareMap.get(Servo.class, "arm"); //Arm
        leftclawServo = myOpMode.hardwareMap.get(Servo.class, "leftClaw");
        rightclawServo = myOpMode.hardwareMap.get(Servo.class, "rightClaw");


        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
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

    public void armRobot(double slidePower, int elbow, boolean claw){
        if(elbow == 0){
            elbowServo.setPosition(0); //Collect
        }
        else if(elbow == 1){
            elbowServo.setPosition(0.3); //Store
        }
        else if(elbow == 2){
            elbowServo.setPosition(0.6); //Storage
        }

        //Claw controls based on bool input
        if(claw){
            closeClaw(leftclawServo, rightclawServo);
        }
        else{
            openClaw(leftclawServo, rightclawServo);
        }

        slides.setPower(slidePower); //set slide power
    }

    public void closeClaw(Servo left, Servo right) {
        left.setPosition(0.4);
        right.setPosition(0.6);
    }

    public void openClaw(Servo left, Servo right) {
        left.setPosition(0.5);
        right.setPosition(0.5);
    }
}