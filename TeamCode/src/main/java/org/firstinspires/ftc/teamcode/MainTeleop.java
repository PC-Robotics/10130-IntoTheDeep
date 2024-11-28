package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Utility.applyDeadzone;
import static org.firstinspires.ftc.teamcode.Utility.clamp;
import static org.firstinspires.ftc.teamcode.Utility.normalizePowers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Controller 1 - Driver
 * joysticks - mecanum drive
 * bumper left - fine control (half sped)
 * options - reset heading
 * trigger left - trolley go in
 * trigger right - trolley go out
 * square - claw close
 * circle - claw open

 * Controller 2 - Operator
 * bumper left - wrist go "in"
 * bumper right - wrist go "out"
 * trigger right - intake spin
 * trigger left - intake spin backwards
 * triangle - bucket release
 * cross - bucket pickup
 * dpadUp - slide index up
 * dpadDown - slide index down
 * left joystick y - slide up down manual control
 */

@TeleOp(name="Main Teleop", group=" Main")
// TODO - figure out subsystems and seperate drive.
public class MainTeleop extends LinearOpMode {
    Robot robot = new Robot(this);
    
    private double straight, turn, strafe, heading;

    private double gamepad1LeftTrigger, gamepad1RightTrigger, gamepad2LeftTrigger, gamepad2RightTrigger, gamepad2RightJoystickY;
    private boolean gamepad2LeftBumper = false, gamepad2RightBumper = false, gamepad2DpadUp = false, gamepad2DpadDown = false, linearSlideInManualMode = false;
    public void runOpMode() {
        robot.init();
        waitForStart();
        robot.imu.resetYaw();

        startingPositions();

        while (opModeIsActive()) {
            readController();
            readSensors();

            robot.driveBase.fieldCentricDrive(straight, strafe, turn, heading, gamepad1.left_bumper);
            linearSlideControl();
            trolleyControl();
            wristControl();
            intakeControl();
            bucketControl();
            clawControl();
        }
    }

    private void startingPositions() {
        robot.trolley.start();
        robot.wrist.start();
        robot.bucket.start();
        robot.claw.start();
        robot.linearSlide.start();
    }

    public void readController() {
        straight = -applyDeadzone(gamepad1.left_stick_y, Settings.DEADZONE_THRESHOLD);
        strafe = applyDeadzone(gamepad1.left_stick_x, Settings.DEADZONE_THRESHOLD);
        turn = applyDeadzone(gamepad1.right_stick_x, Settings.DEADZONE_THRESHOLD);

        gamepad1RightTrigger = applyDeadzone(gamepad1.right_trigger, Settings.DEADZONE_THRESHOLD);
        gamepad1LeftTrigger = applyDeadzone(gamepad1.left_trigger, Settings.DEADZONE_THRESHOLD);
        gamepad2LeftTrigger = applyDeadzone(gamepad2.left_trigger, Settings.DEADZONE_THRESHOLD);
        gamepad2RightTrigger = applyDeadzone(gamepad2.right_trigger, Settings.DEADZONE_THRESHOLD);
        gamepad2RightJoystickY = -applyDeadzone(gamepad2.right_stick_y, Settings.DEADZONE_THRESHOLD);

        if (gamepad1.options) {
            robot.imu.resetYaw();
        }
    }
    private void readSensors() {
        heading = robot.imu.getHeading(AngleUnit.RADIANS);
    }

    private void linearSlideControl() {
        // are we in manual control mode?
        if (gamepad2RightJoystickY != 0) {
            // if we just started manual control this cycle, linearSlideInManualMode will be false
            if (!linearSlideInManualMode) {
                // this means we just started manual control, so we need to set the mode to run without position
                robot.linearSlide.linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                // and set this to true so we don't change the mode over and over again
                linearSlideInManualMode = true;
            }

            // set the power to the joystick value
            robot.linearSlide.linearSlide.setPower(gamepad2RightJoystickY / 2); // half speed for fine control
        } else { // now we're not in manual mode
            // if we're not pressing anything at all, stop the motor with some feedforward
            if (!gamepad2.dpad_up && !gamepad2.dpad_down) {
                robot.linearSlide.stopFeedForward();
            } else { // so we ARE pressing something
                // if we just started position control this cycle, linearSlideInManualMode will be true
                if (linearSlideInManualMode) {
                    // this means we just started position control, so we need to set the mode to run to position
                    robot.linearSlide.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    // and set this to false so we don't change the mode over and over again
                    linearSlideInManualMode = false;
                }

                // if we're pressing dpad up, increase the position index
                if (gamepad2.dpad_up) {
                    if (!gamepad2DpadUp) { // rising edge detection (see game manual 0)
                        robot.linearSlide.increasePosition(Settings.LinearSlide.POWER);
                        gamepad2DpadUp = true;
                    }
                } else {
                    gamepad2DpadUp = false;
                }

                // if we're pressing dpad down, decrease the position index
                // but dpad up takes priority so we don't consider dpad down if dpad up is pressed
                if (gamepad2.dpad_down && !gamepad2.dpad_up) {
                    if (!gamepad2DpadDown) { // rising edge detection (see game manual 0)
                        robot.linearSlide.decreasePosition(Settings.LinearSlide.POWER);
                        gamepad2DpadDown = true;
                    }
                } else {
                    gamepad2DpadDown = false;
                }
            }
        }
    }

    private void trolleyControl() {
        double newTrolleyPosition = robot.trolley.getPosition();

        if (gamepad1RightTrigger != 0) {
            newTrolleyPosition -= (gamepad1RightTrigger / 300);
        } else if (gamepad1LeftTrigger != 0) {
            newTrolleyPosition += (gamepad1LeftTrigger / 300);
        }

        newTrolleyPosition = clamp(newTrolleyPosition, Settings.Trolley.OUT_POSITION, Settings.Trolley.IN_POSITION);
        robot.trolley.setPosition(newTrolleyPosition);
    }

    private void wristControl() {
        if (gamepad2.left_bumper) {
            if (!gamepad2LeftBumper) {
                robot.wrist.moveTowardsPickup();
                gamepad2LeftBumper = true;
            }
        } else {
            gamepad2LeftBumper = false;
        }

        if (gamepad2.right_bumper) {
            if (!gamepad2RightBumper) {
                robot.wrist.moveTowardsRelease();
                gamepad2RightBumper = true;
            }
        } else {
            gamepad2RightBumper = false;
        }
    }

    private void intakeControl() {
        // since the triggers are in the range [0, 1), we can multiply them by the max power to get a percentage of the max power
        if (gamepad2RightTrigger != 0) {
            robot.intake.intake(gamepad2RightTrigger * Settings.Intake.MAX_POWER); // intake the sample
        } else if (gamepad2LeftTrigger != 0) {
            robot.intake.outtake(gamepad2LeftTrigger * Settings.Intake.MAX_POWER); // outtake the sample
        } else {
            robot.intake.stop();
        }
    }

    private void bucketControl() {
        if (gamepad2.triangle) {
            if (robot.linearSlide.positionIndex == 0 && robot.wrist.positionIndex == 2) {
                robot.wrist.moveTowardsPickup();
                robot.bucket.release();
            }
        } else if (gamepad2.cross) {
            robot.bucket.pickup();
        }
    }

    private void clawControl() {
        if (gamepad1.square) {
            robot.claw.close();
        } else if (gamepad1.circle) {
            robot.claw.open();
        }
    }
}
