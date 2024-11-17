package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Utility.applyDeadzone;
import static org.firstinspires.ftc.teamcode.Utility.clamp;
import static org.firstinspires.ftc.teamcode.Utility.normalizePowers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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

    private double[] powers = new double[4];
    private double straight, turn, strafe, heading;

    private double gamepad1LeftTrigger, gamepad1RightTrigger, gamepad2LeftTrigger, gamepad2RightTrigger, gamepad2RightJoystickY;
    private boolean gamepad2LeftBumper = false, gamepad2RightBumper = false, gamepad2DpadUp = false, gamepad2DpadDown = false, linearSlideInManualMode = false;
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
        robot.wrist.setPosition(Settings.Wrist.DRIVING_POSITION);
        robot.bucket.setPosition(Settings.Bucket.PICKUP_POSITION);
        robot.claw.setPosition(Settings.Claw.OPEN_POSITION);

        robot.runLinearSlideToPosition(Settings.LinearSlide.STARTING_POSITION, Settings.LinearSlide.POWER);
        while (robot.linearSlide.isBusy()) {
            sleep(10);
        }
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
        heading = robot.getHeading(); // in radians
    }

    private void mecanumDrive() {
        double rotY = strafe * Math.sin(-heading) + straight * Math.cos(-heading);
        double rotX = strafe * Math.cos(-heading) - straight * Math.sin(-heading);


        // calculate powers
        powers[0] = rotY + rotX + turn; // front left power
        powers[1] = rotY - rotX + turn; // back left power
        powers[2] = rotY - rotX - turn; // front right power
        powers[3] = rotY + rotX - turn; // back right power

        // powers array is updated inside this method
        normalizePowers(powers);

        // set powers to motors
        robot.setMotorPowers(powers);
    }

    private void linearSlideControl() {
        if (gamepad2RightJoystickY != 0) { // fine control
            if (!linearSlideInManualMode) {
                robot.linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            linearSlideInManualMode = true;
            robot.linearSlide.setPower(gamepad2RightJoystickY / 2);
        } else {
            if (linearSlideInManualMode) {
                linearSlideInManualMode = false;
                robot.linearSlide.setPower(0.05);
                robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if (gamepad2.dpad_up) { // run to set positions
                if (!gamepad2DpadUp) {
                    robot.increaseLinearSlidePosition(Settings.LinearSlide.POWER);
                    gamepad2DpadUp = true;
                }
            } else {
                gamepad2DpadUp = false;

                if (gamepad2.dpad_down) {
                    if (!gamepad2DpadDown) {
                        robot.decreaseLinearSlidePosition(Settings.LinearSlide.POWER);
                        gamepad2DpadDown = true;
                    }
                } else {
                    gamepad2DpadDown = false;
                }
            }
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
        if (gamepad1.square) {
            robot.claw.setPosition(Settings.Claw.CLOSED_POSITION);
        } else if (gamepad1.circle) {
            robot.claw.setPosition(Settings.Claw.OPEN_POSITION);
        }
    }

    private void updateTelemetryData() {
        telemetry.addData("Controller Data ", "-----")
                .addData("Gamepad 1 Left Pad Y: ", straight)
                .addData("Gamepad 1 Left Pad X: ", strafe)
                .addData("Gamepad 1 Right Pad X: ", turn)
                .addData("Gamepad 2 Right Pad Y: ", gamepad2RightJoystickY)

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
