package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "sdhunl")
public class GoodAUto extends LinearOpMode {
    Hardware robot;

    @Override
    public void runOpMode() {
        robot = new Hardware(this);
        robot.init();

        waitForStart();


    }

}
