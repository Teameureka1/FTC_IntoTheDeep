package org.firstinspires.ftc.teamcode.References;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Hardware;

/*
 *This Hardware Class is specifically made for Omni POV driving
 *
 * When you use a hardware class in another class, you must:
 *  HardwareClass robot = new HardwareClass(this);
 * Then to use stuff use: robot.
 * For example: robot.init()  To initialize the robot
 */

public class HardwareClass {
    //Declare your OpMode
    private LinearOpMode myOpMode = null;

    /*
     *This is also the spot to set up constant variables like:
     * leftServoClosed = 0.5;
     * leftServoOpen = 0.6;
     */

    //Define Motors (Keep them public to access data for telemetry of HWClass)
    public DcMotor fl = null; //Front Left Drive Motor
    public DcMotor fr = null; //Front Right Drive Motor
    public DcMotor bl = null; //Back Left Drive Motor
    public DcMotor br = null; //Back Right Drive Motor

    /*
     *You should also define any Servos you have here
     * For example:
     * public Servo myServo = null;
     */


    //This is a constructor that allows the OpMode to reference itself
    public HardwareClass(LinearOpMode opMode){myOpMode = opMode;}

    //This function will initialize the robot's hardware (use .init())
    public void init(){
        /*
         * Define your motors as they are in your DS (Drivers Station)
         * So:
         *  motor = myOpMode.hardwareMap.get(DcMotor.class, "Name as seen in DS");
         *  Also do this with your Servo s:
         *  servo = myOpMode.hardwareMap.get(Servo.class, "Name as seen in DS");
         */
        fl = myOpMode.hardwareMap.get(DcMotor.class, "FL"); //Front Left Drive Motor
        fr = myOpMode.hardwareMap.get(DcMotor.class, "FR"); //Front Right Drive Motor
        bl = myOpMode.hardwareMap.get(DcMotor.class, "BL"); //Back Left Drive Motor
        br = myOpMode.hardwareMap.get(DcMotor.class, "BR"); //Back Right Drive Motor

        //Then you want to set your motor directions
        //if your motors don't all spin the same way your bot WILL NOT move right
        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        //When finished, send a message to driver station
        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update(); //And update the telemetry
    }

    //put robot.drive(double, double, double); in your teleop loop and pass gamepad inputs
    public void drive(double axial, double lateral, double yaw){
        //Calculate the power for the wheels
        double flPower = (axial + lateral + yaw); //Front Left Drive Power Calculations
        double frPower = (axial - lateral - yaw); //Front Right Drive Power Calculations
        double blPower = (axial - lateral + yaw); //Back Left Drive Power Calculations
        double brPower = (axial + lateral - yaw); //Back Right Drive Power Calculations
        //Send the calculated power to the motors
        fl.setPower(flPower); //Front Left Drive
        fr.setPower(frPower); //Front Right Drive
        bl.setPower(blPower); //Back Left Drive
        br.setPower(brPower); //Back Right Drive


        //Optional Instant Braking mechanism
        if (axial == 0 && lateral == 0 && yaw == 0) {
            fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);
        }
        else {
            fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

    }
}
