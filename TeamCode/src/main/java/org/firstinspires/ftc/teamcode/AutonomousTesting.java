package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Autonomous Testing", group="Linear OpMode")
public class AutonomousTesting extends LinearOpMode {
    Robot robot = new Robot(this);

    public void runOpMode() {
        robot.init();
        waitForStart();
        robot.imu.resetYaw();

        telemetry.addData("driving", "");
        telemetry.update();
        robot.driveDistance(24, 3000, telemetry);
        sleep(500);

        telemetry.addData("turning", "");
        telemetry.update();
        robot.turnAbsolute(90, telemetry);
        sleep(500);

        telemetry.addData("driving", "");
        telemetry.update();
        robot.driveDistance(24, telemetry);
        sleep(500);

        telemetry.addData("turning", "");
        telemetry.update();
        robot.turnAbsolute(180, telemetry);
        sleep(500);

        telemetry.addData("driving", "");
        telemetry.update();
        robot.driveDistance(24, telemetry);
        sleep(500);

        telemetry.addData("turning", "");
        telemetry.update();
        robot.turnAbsolute(270, telemetry);
        sleep(500);

        telemetry.addData("driving", "");
        telemetry.update();
        robot.driveDistance(24, telemetry);
        sleep(500);

        telemetry.addData("driving", "");
        telemetry.update();
        robot.turnAbsolute(0, telemetry);
        sleep(500);
    }
}
