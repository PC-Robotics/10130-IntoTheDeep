package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Utility.applyDeadzone;
import static org.firstinspires.ftc.teamcode.Utility.clamp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Controller 1 - Driver
 * joysticks - mecanum drive
 * bumper left - fine control (half sped)
 * options - reset heading
 * trigger left - trolley go in (values wrong)
 * trigger right - trolley go out
 * square - claw close (set to bucket now)
 * circle - claw open
 * <p>
 * Controller 2 - Operator
 * bumper left - wrist go "in" (not working well)
 * bumper right - wrist go "out"
 * trigger right - intake spin
 * trigger left - intake spin backwards
 * triangle - bucket release (not working)
 * cross - bucket pickup (slow)
 * dpadUp - slide index up (not working)
 * dpadDown - slide index down
 * left joystick y - slide up down manual control
 */

@TeleOp(name = "Main Teleop", group = " Main")
// TODO - figure out subsystems and seperate drive.
public class MainTeleop extends LinearOpMode {
    Robot robot = new Robot(this);

    private double straight, turn, strafe, heading;

    private double gamepad1LeftTrigger, gamepad1RightTrigger, gamepad2LeftTrigger, gamepad2RightTrigger, gamepad2RightJoystickY;
    private boolean gamepad2LeftBumper = false, gamepad2RightBumper = false, gamepad2DpadUp = false, gamepad2DpadDown = false;

    public void runOpMode() {
        robot.init();
        waitForStart();
        robot.imu.resetYaw();

        startingPositions();

        while (opModeIsActive()) { // hi
            readController();
            readSensors();

            robot.driveBase.fieldCentricDrive(straight, strafe, turn, heading, gamepad1.left_bumper);
            linearSlideControl();
            // trolleyControl();
            wristControl();
            intakeControl();
            bucketControl();
            clawControl();

            reloadTelemetry();
            telemetry.update();
        }
    }

    private void startingPositions() {
        // robot.trolley.start();
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

    /**
     * Controls the linear slide mechanism with two modes: Position Mode and Manual Mode.
     * <p>
     * Position Mode:
     * - Triggered by pressing dpad up or dpad down on gamepad2.
     * - Moves the slide to pre-defined positions using preset methods.
     * <p>
     * Manual Mode:
     * - Triggered when gamepad2's right joystick y-axis is used.
     * - Allows direct control of the slide motor with clamped power levels.
     * - Prevents the slide from moving beyond STARTING_POSITION and SECOND_BUCKET_POSITION.
     * - Temporarily switches the motor to RUN_USING_ENCODER mode.
     * <p>
     * If no input is detected, a minimal feed-forward power is applied to hold position.
     */
    double scaledManualPower, clampedPower;

    private void linearSlideControl() {
        scaledManualPower = gamepad2RightJoystickY / 2; // Scaled joystick input for finer control

        // Rising edge detection for dpad_up
        if (gamepad2.dpad_up && !gamepad2DpadUp) {
            gamepad2DpadUp = true;
            if (robot.linearSlide.positionIndex == 0 && robot.wrist.positionIndex == 2) {
                robot.wrist.moveToPositionIndex(1);
                sleep(50);
            }
            robot.linearSlide.increasePosition(Settings.LinearSlide.POWER);
        } else if (!gamepad2.dpad_up) {
            gamepad2DpadUp = false;
        }

        // Rising edge detection for dpad_down
        if (gamepad2.dpad_down && !gamepad2DpadDown) {
            gamepad2DpadDown = true;
            if (robot.linearSlide.positionIndex == 1 && robot.wrist.positionIndex == 2) {
                robot.wrist.moveToPositionIndex(1);
                sleep(50);
            }
            robot.linearSlide.decreasePosition(Settings.LinearSlide.POWER);
        } else if (!gamepad2.dpad_down) {
            gamepad2DpadDown = false;
        }

        // Manual control
        if (!gamepad2.dpad_up && !gamepad2.dpad_down && scaledManualPower != 0) {
            clampedPower = clamp(scaledManualPower, -Settings.LinearSlide.FINE_CONTROL_POWER, Settings.LinearSlide.FINE_CONTROL_POWER);
            robot.linearSlide.linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (
                    (robot.linearSlide.linearSlide.getCurrentPosition() > Settings.LinearSlide.POSITIONS.get(0) && clampedPower < 0) ||
                            (robot.linearSlide.linearSlide.getCurrentPosition() < Settings.LinearSlide.POSITIONS.get(Settings.LinearSlide.POSITIONS.length()) && clampedPower > 0)
            ) {
                robot.linearSlide.linearSlide.setPower(clampedPower);
            } else {
                robot.linearSlide.linearSlide.setPower(0);
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
        if (gamepad2.left_bumper && !gamepad2LeftBumper) {
            robot.wrist.moveTowardsPickup();
            gamepad2LeftBumper = true;
        } else if (!gamepad2.left_bumper) {
            gamepad2LeftBumper = false;
        }

        if (gamepad2.right_bumper && !gamepad2RightBumper) {
            robot.wrist.moveTowardsRelease();
            gamepad2RightBumper = true;
        } else if (!gamepad2.right_bumper) {
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
                robot.wrist.moveToPositionIndex(1);
                sleep(100);
            }
            robot.bucket.release();
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

    private void reloadTelemetry() {
        robot.trolley.telemetry();
        robot.wrist.telemetry();
        robot.bucket.telemetry();
        robot.claw.telemetry();
        robot.linearSlide.telemetry();
        robot.intake.telemetry();

        telemetry.addData("DRIVE BASE", "-----------");
        robot.imu.telemetry();
        robot.driveBase.telemetry();

        telemetry.update();
    }
}
