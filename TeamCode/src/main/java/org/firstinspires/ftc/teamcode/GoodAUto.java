package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "create")
public class GoodAUto extends LinearOpMode {
    Hardware robot;

    @Override
    public void runOpMode() {
        robot = new Hardware(this);
        robot.init();
        int config = 2;

        waitForStart();

        if(config == 1){
            sleep(1000);
            robot.drive(0, -1, 0, 0.4);
            sleep(2750);
            robot.drive(0,1,0,0.4);
            sleep(7000);
            robot.drive(0, 1, 0, 1);
            sleep(5000);
        }
        if(config == 2){
            robot.drive(0, 1, 0, 0.4);
            sleep(3000);
            robot.drive(0,0,0,0);
        }











    }



}
