package org.firstinspires.ftc.teamcode.drivebase;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.Utility.applyDeadzone;
import static org.firstinspires.ftc.teamcode.Utility.normalizePowers;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name="Mecanum Drive", group="Drivebase")
public class TeleopMecanumDrive extends LinearOpMode {
    private static final double DEADZONE_THRESHOLD = 0.1;

    Robot robot = new Robot(this);

    private double[] powers = new double[4];
    double straight, turn, strafe, heading;

    public void runOpMode() {
        robot.init();
        waitForStart();
        robot.imu.resetYaw();

        while (opModeIsActive()) {
            readSensors();
            mecanumDrive(straight, turn, strafe);
            updateTelemetryData();
        }
    }

    private void readSensors() {
        straight = applyDeadzone(gamepad1.left_stick_y, DEADZONE_THRESHOLD);
        turn = applyDeadzone(gamepad1.right_stick_x, DEADZONE_THRESHOLD);
        strafe = applyDeadzone(-gamepad1.left_stick_x, DEADZONE_THRESHOLD); // invert pad left x
        heading = robot.getHeading(); // in radians
    }

    private void mecanumDrive(double straight, double turn, double strafe) {
        // calculate powers
        powers[0] = straight + turn + strafe; // front left power
        powers[1] = straight + turn - strafe; // back left power
        powers[2] = straight - turn - strafe; // front right power
        powers[3] = straight - turn + strafe; // back right power

        // powers array is updated inside this method
        normalizePowers(powers);

        // set powers to motors
        robot.frontLeft.setPower(powers[0]);
        robot.backLeft.setPower(powers[1]);
        robot.frontRight.setPower(powers[2]);
        robot.backRight.setPower(powers[3]);
    }

    private void updateTelemetryData() {
        telemetry.addData("Left Pad Y", straight)
                .addData("Left Pad X", strafe)
                .addData("Right Pad X", turn)
                .addData("Front Left Power", powers[0])
                .addData("Back Left Power", powers[1])
                .addData("Front Right Power", powers[2])
                .addData("Back Right Power", powers[3])
                .addData("Heading", Math.toDegrees(heading) + 180);
        telemetry.update();
    }
}
