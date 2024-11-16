package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import java.text.DecimalFormat;

@TeleOp(name="Servo Position Locator",group="Testing")

public class ServoPositionLocator extends LinearOpMode {

    //
    //
    //
    //
    // NOTE - USER EDITABLE VARIABLES

    // Change this to the name of the servo on the hardware map
    static final String SERVO_NAME = "left";
    static final String SERVO_NAME2 = "right";

    // INCREMENTS - Calculated as a percentage of the servo's total range
    // On a 5-turn servo, 0.01 will move the servo more than on a 1-turn servo

    // Small increment for fine tuning
    static final double SMALL_INCREMENT = 0.01;

    // Large increment for large adjustments
    static final double LARGE_INCREMENT = 0.1;

    // The maximum and minimum positions of YOUR INTENDED RANGE OF THE SERVO
    static final double MAX_POS = 1.0;
    static final double MIN_POS = 0.0;

    // This is the position the servo will move to when the program starts
    // By default, it is the middle of the range
    // You can edit it to a specific position if you want
    static final double STARTING_POS = (MAX_POS + MIN_POS) / 2;

    // NOTE - END OF USER EDITABLE VARIABLES
    //
    //
    //
    //

    // pretty printing stuff
    DecimalFormat df = new DecimalFormat("#.####");

    private double position = STARTING_POS;

    // These booleans are used to ensure that each button press is only counted once
    // Cannot hold the button for these operations
    private boolean xPressed = false;
    private boolean bPressed = false;
    private boolean leftPressed = false;
    private boolean rightPressed = false;

    @Override
    public void runOpMode() {
        // Connect to the servo
        // Change device name in variable definitions
        // Can move to an FTC Dashboard Config constants file to be able to change there
        Servo servo = hardwareMap.get(Servo.class, SERVO_NAME);
        Servo servo2 = hardwareMap.get(Servo.class, SERVO_NAME2);

        servo.setDirection(Servo.Direction.REVERSE);
        servo.setDirection(Servo.Direction.REVERSE);

        telemetry.addData(">", "Press Start to test "+ SERVO_NAME);
        telemetry.addData(">", "Use x and b to make small adjustments");
        telemetry.addData(">", "Use dpad left and right to make large adjustments");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()) {
            // Use x and b on Gamepad 1 to make smaller servo adjustments

            // x moves the servo to the left
            if(gamepad1.x) {
                if(!xPressed) {
                    xPressed = true;
                    position -= SMALL_INCREMENT;
                }
            } else {
                xPressed = false;
            }

            // b moves the servo to the right
            if(gamepad1.b) {
                if(!bPressed) {
                    bPressed = true;
                    position += SMALL_INCREMENT;
                }
            } else {
                bPressed = false;
            }

            // Use Left and Right on d-pad for large adjustments

            // left moves the servo to the left
            if(gamepad1.dpad_left) {
                if(!leftPressed) {
                    leftPressed = true;
                    position -= LARGE_INCREMENT;
                }
            } else {
                leftPressed = false;
            }

            // right moves the servo to the right
            if(gamepad1.dpad_right) {
                if(!rightPressed) {
                    rightPressed = true;
                    position += LARGE_INCREMENT;
                }
            } else {
                rightPressed = false;
            }

            // Clamp the position to the range of the servo
            position = Utility.clamp(position, MIN_POS, MAX_POS);

            // Set the servo to the new position
            servo.setPosition(position);
            servo2.setPosition(position);

            telemetry.addData("Servo Position", df.format(position));
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

        }
    }
}