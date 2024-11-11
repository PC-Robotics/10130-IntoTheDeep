package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Utility.applyDeadzone;
import static org.firstinspires.ftc.teamcode.Utility.clamp;
import static org.firstinspires.ftc.teamcode.Utility.normalizePowers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Controller 1 - Driver
 * joysticks - mecanum drive
 * bumper left - fine control (half sped)
 * dpad up - linear slide go up
 * dpad down - linear slide go down
 * trigger right - arm go out
 * trigger left - arm go in

 * Controller 2 - Operator
 * bumper left - wrist go left
 * bumper right - wrist go right
 * trigger right - intake spin
 * trigger left - intake spin backwards
 * triangle - bucket release
 * cross - bucket pickup
 * dpad left - claw close
 * dpad right - claw open
 */

@TeleOp(name="Main Teleop Program", group="Linear OpMode")
// TODO - figure out subsystems and seperate drive.
public class MainTeleop extends LinearOpMode {
    Robot robot = new Robot(this);

    private double[] powers = new double[4];
    private double straight, turn, strafe, heading;

    private double gamepad1LeftTrigger, gamepad1RightTrigger, gamepad2LeftTrigger, gamepad2RightTrigger;
    private boolean gamepad1DpadUp = false, gamepad1DpadDown = false, gamepad2LeftBumper = false, gamepad2RightBumper = false;
    public void runOpMode() {
        robot.init();
        waitForStart();
        robot.imu.resetYaw();

        initOpMode();

        while (opModeIsActive()) {
            readController();
            readSensors();

            mecanumDrive();
            linearSlideControl();
            trolleyControl();
            wristControl();
            intakeControl();
            bucketControl();
            clawControl();

            updateTelemetryData();
        }
    }

    private void initOpMode() {
        robot.setTrolleyPosition(Settings.Trolley.IN_POSITION);
        robot.wrist.setPosition(Settings.Wrist.RELEASE_POSITION);
        robot.bucket.setPosition(Settings.Bucket.PICKUP_POSITION);
        robot.claw.setPosition(Settings.Claw.OPEN_POSITION);

        robot.runLinearSlideToPosition(Settings.LinearSlide.STARTING_POSITION, Settings.LinearSlide.POWER);
        while (robot.linearSlide.isBusy()) {
            sleep(10);
        }
    }

    public void readController() {
        straight = applyDeadzone(gamepad1.left_stick_y, Settings.DEADZONE_THRESHOLD);
        strafe = -applyDeadzone(gamepad1.left_stick_x, Settings.DEADZONE_THRESHOLD);
        turn = applyDeadzone(gamepad1.right_stick_x, Settings.DEADZONE_THRESHOLD);

        gamepad1RightTrigger = applyDeadzone(gamepad1.right_trigger, Settings.DEADZONE_THRESHOLD);
        gamepad1LeftTrigger = applyDeadzone(gamepad1.left_trigger, Settings.DEADZONE_THRESHOLD);
        gamepad2LeftTrigger = applyDeadzone(gamepad2.left_trigger, Settings.DEADZONE_THRESHOLD);
        gamepad2RightTrigger = applyDeadzone(gamepad2.right_trigger, Settings.DEADZONE_THRESHOLD);
    }
    private void readSensors() {
        heading = robot.getHeading(); // in radians
    }

    private void mecanumDrive() {
        // calculate powers
        powers[0] = straight + strafe - turn; // front left power
        powers[1] = straight - strafe - turn; // back left power
        powers[2] = straight - strafe + turn; // front right power
        powers[3] = straight + strafe + turn; // back right power

        // fine control
        if (gamepad1.left_bumper) {
            for (int i = 0; i < powers.length; i++) {
                powers[i] /= 2;
            }
        }

        // powers array is updated inside this method
        powers = normalizePowers(powers);

        // set powers to motors
        robot.setMotorPowers(powers);
    }

    private void linearSlideControl() {
        if (gamepad1.dpad_left || gamepad1.dpad_right) { // fine control
            if (gamepad1.dpad_left && gamepad1.dpad_right) {
                robot.releaseSpecimen();
            } else {
                int newLinearSlidePosition = robot.linearSlide.getCurrentPosition();
                if (gamepad1.dpad_left) {
                    newLinearSlidePosition += 50;
                } else {
                    newLinearSlidePosition -= 50;
                }

                newLinearSlidePosition = clamp(newLinearSlidePosition, Settings.LinearSlide.STARTING_POSITION, Settings.LinearSlide.SECOND_BUCKET_POSITION);
                robot.runLinearSlideToPosition(newLinearSlidePosition, Settings.LinearSlide.FINE_CONTROL_POWER);
            }

        } else if (gamepad1.dpad_up) { // run to set positions
            if (!gamepad1DpadUp) {
                robot.increaseLinearSlidePosition(Settings.LinearSlide.POWER);
                gamepad1DpadUp = true;
            }
        } else {
            gamepad1DpadUp = false;
        }

        if (gamepad1.dpad_down && !gamepad1.dpad_up) {
            if (!gamepad1DpadDown) {
                robot.decreaseLinearSlidePosition(Settings.LinearSlide.POWER);
                gamepad1DpadDown = true;
            }
        } else {
            gamepad1DpadDown = false;
        }

        if (!robot.linearSlide.isBusy()) {
            robot.linearSlide.setPower(0.1);
        }
    }

    private void trolleyControl() {
        double newTrolleyPosition = robot.getTrolleyPosition();

        if (gamepad1RightTrigger != 0) {
            newTrolleyPosition -= (gamepad1RightTrigger / 300);
        } else if (gamepad1LeftTrigger != 0) {
            newTrolleyPosition += (gamepad1LeftTrigger / 300);
        }

        newTrolleyPosition = clamp(newTrolleyPosition, Settings.Trolley.OUT_POSITION, Settings.Trolley.IN_POSITION);
        robot.setTrolleyPosition(newTrolleyPosition);
    }

    private void wristControl() {
        if (gamepad2.left_bumper) {
            if (!gamepad2LeftBumper) {
                robot.increaseWristPosition();
                gamepad2LeftBumper = true;
            }
        } else {
            gamepad2LeftBumper = false;
        }

        if (gamepad2.right_bumper) {
            if (!gamepad2RightBumper) {
                robot.decreaseWristPosition();
                gamepad2RightBumper = true;
            }
        } else {
            gamepad2RightBumper = false;
        }
    }

    private void intakeControl() {
        // multiplying by Settings.MAX_INTAKE_POWER normalizes the trigger values to [0, MAX_INTAKE_POWER]
        if (gamepad2RightTrigger != 0) {
            robot.intake.setPower(gamepad2RightTrigger * Settings.Intake.MAX_POWER);
        } else if (gamepad2LeftTrigger != 0) {
            robot.intake.setPower(-(gamepad2LeftTrigger * Settings.Intake.MAX_POWER));
        } else {
            robot.intake.setPower(0);
        }
    }

    private void bucketControl() {
        if (gamepad2.triangle) {
            if (!(robot.linearSlide.getTargetPosition() == Settings.LinearSlide.STARTING_POSITION && robot.wrist.getPosition() == Settings.Wrist.RELEASE_POSITION)) {
                robot.bucket.setPosition(Settings.Bucket.RELEASE_POSITION);
            }
        } else if (gamepad2.cross) {
            robot.bucket.setPosition(Settings.Bucket.PICKUP_POSITION);
        }
    }

    private void clawControl() {
        if (gamepad2.dpad_left) {
            robot.claw.setPosition(Settings.Claw.CLOSED_POSITION);
        } else if (gamepad2.dpad_right) {
            robot.claw.setPosition(Settings.Claw.OPEN_POSITION);
        }
    }

    private void updateTelemetryData() {
        telemetry.addData("Controller Data ", "-----")
                .addData("Left Pad Y: ", straight)
                .addData("Left Pad X: ", strafe)
                .addData("Right Pad X: ", turn)

                .addData("Drive Data ", "-----")
                .addData("Front Left Power: ", powers[0])
                .addData("Back Left Power: ", powers[1])
                .addData("Front Right Power: ", powers[2])
                .addData("Back Right Power: ", powers[3])
                .addData("Heading: ", Math.toDegrees(heading) + 180)

                .addData("Subsystem Data ", "-----")
                .addData("Slide Position Index: ", robot.getLinearSlideIndex())
                .addData("Slide Position", robot.linearSlide.getCurrentPosition())
                .addData("Wrist Position Index: ", robot.getWristIndex())
                .addData("Arm Position: ", robot.getTrolleyPosition());

        telemetry.update();
    }
}
