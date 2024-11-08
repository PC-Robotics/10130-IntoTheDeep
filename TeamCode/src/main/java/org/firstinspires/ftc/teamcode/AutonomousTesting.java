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

        robot.driveDistance(24, telemetry);
        robot.turnAbsolute(90, telemetry);
        robot.driveDistance(24, telemetry);
        robot.turnAbsolute(180, telemetry);
        robot.driveDistance(24, telemetry);
        robot.turnAbsolute(270, telemetry);
        robot.driveDistance(24, telemetry);
        robot.turnAbsolute(0, telemetry);
    }
}
