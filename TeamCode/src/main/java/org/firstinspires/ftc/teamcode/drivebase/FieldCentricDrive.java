package org.firstinspires.ftc.teamcode.drivebase;

import static org.firstinspires.ftc.teamcode.Utility.applyDeadzone;
import static org.firstinspires.ftc.teamcode.Utility.normalizePowers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name="Field Centric Drive", group="DriveBase")
public class FieldCentricDrive extends LinearOpMode {
    private static final double DEADZONE_THRESHOLD = 0.1;

    Robot robot = new Robot(this);

    private double[] powers = new double[4];
    double straight, turn, strafe, heading;

    public void runOpMode() {
        robot.init();
        waitForStart();
        robot.imu.resetYaw();

        while (opModeIsActive()) {
            if (gamepad1.options) {
                robot.imu.resetYaw();
            }

            readSensors();
            FieldCentricDrive();
            updateTelemetryData();
        }
    }

    private void readSensors() {
        straight = -applyDeadzone(gamepad1.left_stick_y, DEADZONE_THRESHOLD);
        turn = applyDeadzone(gamepad1.right_stick_x, DEADZONE_THRESHOLD);
        strafe = applyDeadzone(gamepad1.left_stick_x, DEADZONE_THRESHOLD); // invert pad left x
        heading = robot.getHeading(); // in radians
    }

    /**
     * @see <a href="https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html">gm0 guide</a>
     */
    private void FieldCentricDrive() {
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
