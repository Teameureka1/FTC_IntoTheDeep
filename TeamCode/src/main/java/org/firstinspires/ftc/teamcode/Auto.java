package org.firstinspires.ftc.teamcode;

import android.os.Bundle;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@Autonomous(name="AutoBad", group="Robot")


public class Auto extends LinearOpMode {
    RobotHardwareBruce robot = new RobotHardwareBruce(this); //use robot.
    private ElapsedTime runtime = new ElapsedTime(); //define runtime

    static final double     FORWARD_SPEED = 0;
    static final double     TURN_SPEED    = 0.4;
    int yaw;

    @Override
    public void runOpMode(){
        robot.init(); //Initialize the robot

        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart(); // Wait for driver to press START

        //robot.abc(50);
        forward(1000);
        // Start robot driving here
        //PLACE HOLDER TIMES USE CAUTION
        //right(1);
        //forward(1);
        //Grab drop Grab Drop Grab Drop

        //sleep(100);

        //right(2);
        //rotate(2, 0);
        //forward(1);


    }

    //Remember: robot.closeClaw() and robot.openClaw()


    public void slidePower(int direction, double timeMoving){
        if(direction == 0){
            //robot.slide(0.7);
        }
        else{
            //robot.slide(-0.7);
        }
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < timeMoving)){
            //do telementry stuff here
        }

    }

    public void forward(double timeMoving){
        robot.driveRobot(1,0,0,FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < timeMoving)) {
            telemetry.addData("Driving Forward", "Placeholder: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }

    public void backward(double timeMoving){
        robot.driveRobot(-1,0,0,FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < timeMoving)) {
            telemetry.addData("Driving Backward", "Placeholder: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }

    public void left(double timeMoving){
        robot.driveRobot(0,1,0,FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < timeMoving)) {
            telemetry.addData("Driving Left", "Placeholder: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }

    public void right(double timeMoving){
        robot.driveRobot(0,-1,0,FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < timeMoving)) {
            telemetry.addData("Driving Right", "Placeholder: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }

    public void rotate(double seconds, int direction){
        if (direction == 0){
            yaw = -1;
        }
        else{
            yaw = 1;
        }
        robot.driveRobot(0,0, yaw, TURN_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < seconds)) { //1 second for rotation
            telemetry.addData("Rotating!", "Placeholder: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }

}
