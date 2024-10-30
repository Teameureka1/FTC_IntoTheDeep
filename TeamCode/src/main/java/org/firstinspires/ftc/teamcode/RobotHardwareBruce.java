package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * This file works in conjunction with the External Hardware Class sample called: NewTeleOp.java
 * Please read the explanations in that Sample about how to use this class definition.
 *
 * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware (motors and sensors).
 * It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with *exactly the same name*.
 *
 * Or... In OnBot Java, add a new file named RobotHardware.java, select this sample, and select Not an OpMode.
 * Also add a new OpMode, select the sample NewTeleOp.java, and select TeleOp.
 *
 */

public class RobotHardwareBruce {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    public DcMotor leftFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor slides = null;

    public Servo elbowServo = null;
    public Servo leftclawServo = null;
    public Servo rightclawServo = null;
    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position
    double  position = 0; // Start at halfway position

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    public static final double MID_SERVO       =  0.5 ;
    public static final double HAND_SPEED      =  0.02 ;  // sets rate to move servo
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    double posleft = 1;
    double posright = 0;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardwareBruce(LinearOpMode opMode) {
        myOpMode = opMode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init()    {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        leftFrontDrive  = myOpMode.hardwareMap.get(DcMotor.class, "FL"); //Front Left
        leftBackDrive  = myOpMode.hardwareMap.get(DcMotor.class, "FR"); //Front Right
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "BL"); //Back Left
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "BR"); //Back Right
        slides = myOpMode.hardwareMap.get(DcMotor.class, "slide"); //Slide
        elbowServo = myOpMode.hardwareMap.get(Servo.class, "arm"); //Arm
        leftclawServo = myOpMode.hardwareMap.get(Servo.class, "leftClaw");
        rightclawServo = myOpMode.hardwareMap.get(Servo.class, "rightClaw");

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos. EXAMPLE
        //leftHand = myOpMode.hardwareMap.get(Servo.class, "left_hand");
        //leftHand.setPosition(MID_SERVO);
        leftclawServo.setPosition(0.35);
        rightclawServo.setPosition(0.65);


        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }


    public void driveRobot(double axial, double lateral, double yaw,
                           double slideForward, double slideBack, double claw,
                           int elbow, boolean buttonX, boolean buttonB) {

        double max;
        double leftClosed = 0.35;
        double rightClosed = 0.65;
        // Combine drive and turn for blended motion.


        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;
        double slidePower = slideForward + slideBack;
        double clawPower = claw;


        if(elbow == 0){
            elbowServo.setPosition(0.0); //Collect
        }
        else if(elbow == 1){
            elbowServo.setPosition(0.3); //Store
        }
        else if(elbow == 2){
            elbowServo.setPosition(1); //Storage
        }

        if(buttonX){
            closeClaw(leftclawServo, rightclawServo);
        }
        if(buttonB){
            openClaw(leftclawServo, rightclawServo);
        }

        //leftclaw.setPosition(0);
        //rightclaw.setPosition(0);

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        slides.setPower(slidePower);


    }

    public void closeClaw(Servo left, Servo right) {
        left.setPosition(0.35);
        right.setPosition(0.65);
    }

    public void openClaw(Servo left, Servo right) {
        left.setPosition(0.5);
        right.setPosition(0.5);
    }

    /**
     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
     *
     * @param *leftWheel     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param *rightWheel    Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     */
    public void setDrivePower(double leftFrontPower, double rightFrontPower,
                              double leftBackPower, double rightBackPower) {
        // Output the values to the motor drives.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }


    /**
     * Pass the requested arm power to the appropriate hardware drive motor
     *
     * @param power driving power (-1.0 to 1.0)
     *
    public void setArmPower(double power) {
        armMotor.setPower(power);
    }

    /**
     * Send the two hand-servos to opposing (mirrored) positions, based on the passed offset.
     *
     * @param offset
     *
    public void setHandPositions(double offset) {
        offset = Range.clip(offset, -0.5, 0.5);
        leftHand.setPosition(MID_SERVO + offset);
        rightHand.setPosition(MID_SERVO - offset);

    }*/
}