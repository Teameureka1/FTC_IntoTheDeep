package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="TeleOp", group="Robot")

public class NewTeleOp extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    RobotHardwareBruce robot       = new RobotHardwareBruce(this);

    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {


        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();
        int elbow = 3;
        boolean claw = true;
        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;
            double slidePower = -gamepad2.left_stick_y;
            double speed = gamepad1.right_trigger;


            if (gamepad2.left_bumper){//open
                claw = false;
            }
            if (gamepad2.right_bumper){//closed
                claw = true;
            }

            if(gamepad2.dpad_left){ //up
                elbow = 2;
            }
            if(gamepad2.dpad_up){ //middle
                elbow = 1;
            }
            if(gamepad2.dpad_right){ //down
                elbow = 0;
            }


            // Use RobotHardware Class
            robot.driveRobot(axial, lateral, yaw, speed); //Drive robot
            robot.armRobot(slidePower, elbow, claw); //Arm operations


            // Send telemetry messages to explain controls and show robot status
            telemetry.addData("Status", "Run Time: " + runtime.toString());


            telemetry.addData("Drive", "Left Stick");
            telemetry.addData("Turn", "Right Stick");
            telemetry.addData("Arm Up/Down", "Y & A Buttons");
            telemetry.addData("Hand Open/Closed", "Left and Right Bumpers");
            telemetry.addData("-", "-------");
            telemetry.addData("Axial", "%.2f", axial);
            telemetry.addData("Lateral", "%.2f", lateral);
            telemetry.addData("Yaw", "%.2f", yaw);
            telemetry.addData("Power", "%.2f", speed);
/*
            telemetry.addData("Drive Power", "%.2f", );
            telemetry.addData("Turn Power",  "%.2f", turn);
            telemetry.addData("Arm Power",  "%.2f", arm);
            telemetry.addData("Hand Position",  "Offset = %.2f", handOffset);*/
            telemetry.update();

            // Pace this loop so hands move at a reasonable speed.
            sleep(50);
        }
    }
}