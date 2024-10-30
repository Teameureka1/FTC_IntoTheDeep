package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="TeleOp Hardware Test", group="Robot")

public class NewTeleOp extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    RobotHardwareBruce robot       = new RobotHardwareBruce(this);

    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {


        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();

        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = gamepad1.left_stick_x;  // Note: pushing stick forward gives negative value
            double lateral = -gamepad1.left_stick_y;
            double yaw     =  gamepad1.right_stick_x;
            double slideForward = gamepad2.left_stick_y;
            double slideBack = -gamepad2.left_stick_y;
            int elbow = 2;
            //double armTransport =
            //double armCollect =
            double claw = gamepad2.right_stick_x;
            boolean buttonX = gamepad2.x;
            boolean buttonB = gamepad2.b;

            if(gamepad2.dpad_down){
                if(elbow != 0){
                    elbow -= 1;
                }
            }
            if(gamepad2.dpad_up){
                if(elbow != 2){
                    elbow += 1;
                }
            }

            // Combine drive and turn for blended motion. Use RobotHardware class
            robot.driveRobot(axial, lateral, yaw, slideForward, slideBack, claw, elbow, buttonX, buttonB);

            /*
            // Use gamepad left & right Bumpers to open and close the claw
            // Use the SERVO constants defined in RobotHardware class.
            // Each time around the loop, the servos will move by a small amount.
            // Limit the total offset to half of the full travel range
            if (gamepad1.right_bumper)
                handOffset += robot.HAND_SPEED;
            else if (gamepad1.left_bumper)
                handOffset -= robot.HAND_SPEED;
            handOffset = Range.clip(handOffset, -0.5, 0.5);

            // Move both servos to new position.  Use RobotHardware class
            robot.setHandPositions(handOffset);

            // Use gamepad buttons to move arm up (Y) and down (A)
            // Use the MOTOR constants defined in RobotHardware class.
            if (gamepad1.y)
                arm = robot.ARM_UP_POWER;
            else if (gamepad1.a)
                arm = robot.ARM_DOWN_POWER;
            else
                arm = 0;

            robot.setArmPower(arm);*/

            // Send telemetry messages to explain controls and show robot status
            telemetry.addData("Status", "Run Time: " + runtime.toString());


            telemetry.addData("Drive", "Left Stick");
            telemetry.addData("Turn", "Right Stick");
            telemetry.addData("Arm Up/Down", "Y & A Buttons");
            telemetry.addData("Hand Open/Closed", "Left and Right Bumpers");
            telemetry.addData("-", "-------");
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