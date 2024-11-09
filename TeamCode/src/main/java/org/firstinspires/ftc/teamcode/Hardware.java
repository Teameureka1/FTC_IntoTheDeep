package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Hardware {
    //Declare OpMode
    private LinearOpMode myOpMode = null;

    //Claw Positions

    public double leftOpen = 0.4;
    public double rightOpen = 0.6;

    public double leftClosed1 = 0.53;
    public double rightClosed1 = 0.47;
    public double leftClosed2 = 0.6;
    public double rightClosed2 = 0.4;

    public int storagePosition = -90;
    public int upRight = -45;
    public int level = 24;
    public int collectPosition = 31;

    //Define Motors and Servos
    public DcMotor fl = null;
    public DcMotor fr = null;
    public DcMotor bl = null;
    public DcMotor br = null;

    public DcMotor slideMotor = null;
    public DcMotor armMotor = null;

    public Servo leftClawServo = null;
    public Servo rightClawServo = null;

    // Constructor to allow OpMode to reference itself
    public Hardware(LinearOpMode opMode){myOpMode = opMode;}

    //Initialize all the robot's hardware
    public void init(){
        //Define / Initialize Motors
        fl = myOpMode.hardwareMap.get(DcMotor.class, "FL"); //Front Left
        fr = myOpMode.hardwareMap.get(DcMotor.class, "FR"); //Front Right
        bl = myOpMode.hardwareMap.get(DcMotor.class, "BL"); //Back Left
        br = myOpMode.hardwareMap.get(DcMotor.class, "BR"); //Back Right
        slideMotor = myOpMode.hardwareMap.get(DcMotor.class, "slide");
        armMotor = myOpMode.hardwareMap.get(DcMotor.class, "arm");

        //Set motor directions
        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);
        slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);


        // Define / Initialize Servos
        leftClawServo = myOpMode.hardwareMap.get(Servo.class, "leftClaw");
        rightClawServo = myOpMode.hardwareMap.get(Servo.class, "rightClaw");


        myOpMode.telemetry.addData(">", "Hardware Initialized");

        myOpMode.telemetry.addData("Left Pos: ", "%2f", leftClawServo.getPosition());
        myOpMode.telemetry.addData("Right Pos: ", "%2f", rightClawServo.getPosition());

        myOpMode.telemetry.update();
    }

    public void drive(double axial, double lateral, double yaw, double speed){
        // Calculate wheel power
        double flPower = (axial + lateral + yaw) * speed;
        double frPower = (axial - lateral - yaw) * speed;
        double blPower = (axial - lateral + yaw) * speed;
        double brPower = (axial + lateral - yaw) * speed;
        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);

        if (axial == 0 && lateral == 0 && yaw == 0) {
            fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);
        } else {
            fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    public void arm(double slidePower, int claw, double armPower){
        if(claw == 0){ //Open
            leftClawServo.setPosition(leftOpen);
            rightClawServo.setPosition(rightOpen);
        }
        else if(claw == 1){ //closed big
            leftClawServo.setPosition(leftClosed1);
            rightClawServo.setPosition(rightClosed1);
        }
        else if(claw == 2){ //closed small
            leftClawServo.setPosition(leftClosed2);
            rightClawServo.setPosition(rightClosed2);
        }
        slideMotor.setPower(slidePower);


        myOpMode.telemetry.addData("Arm Pos: ", armMotor.getCurrentPosition());
    }

}
