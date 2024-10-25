package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Servo Location Finder",group="Testing")

public class ServoPositionLocator extends LinearOpMode {

    static final String TESTED_SERVO = "testServo";

    static final double SMALL_INCREEMENT = 0.01;
    static final double LARGE_INCREMENT = 0.1;
    static final double MAX_POS = 1.0;
    static final double MIN_POS = 0.0;

    static final double MIDDLE_POS = (MAX_POS + MIN_POS)/2;

    private double position = MIDDLE_POS;

    private Servo servo = null;


    // These booleans are used to ensure that each button press is only counted once
    // Cannot hold the button for these operations
    private boolean xPressed = false;
    private boolean bPressed = false;
    private boolean rightPressed = false;
    private boolean leftPressed = false;


    @Override
    public void runOpMode() throws InterruptedException {
        // Connect to the servo
        // Change device name in variable definitions
        // Can move to an FTC Dashboard Config constants file to be able to change there
        servo = hardwareMap.get(Servo.class, TESTED_SERVO);

        telemetry.addData(">", "Press Start to test "+TESTED_SERVO );
        telemetry.update();
        waitForStart();

        while(opModeIsActive())
        {

            // Use x and b on Gamepad 1 to make smaller servo adjustments
            if(gamepad1.x)
            {
                if(!xPressed)
                {
                    xPressed = true;
                    position -= SMALL_INCREEMENT;
                }
            }
            else
            {
                xPressed = false;
            }

            if(gamepad1.b)
            {
                if(!bPressed)
                {
                    bPressed = true;
                    position += SMALL_INCREEMENT;
                }
            }
            else
            {
                bPressed = false;
            }

            // Use Left and Right on Gamepad 1's d-pad for large adjustments
            if(gamepad1.dpad_left)
            {
                if(!leftPressed)
                {
                    leftPressed = true;
                    position -= LARGE_INCREMENT;
                }
            }
            else
            {
                leftPressed = false;
            }

            if(gamepad1.dpad_right)
            {
                if(!rightPressed)
                {
                    rightPressed = true;
                    position += LARGE_INCREMENT;
                }
            }
            else
            {
                rightPressed = false;
            }

            servo.setPosition(position);

            telemetry.addData("Servo Position", "%5.2f", position);
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

        }
    }
}