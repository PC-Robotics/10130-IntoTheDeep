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

        robot.driveDistance(24);
        robot.turnAbsolute(90);
        robot.driveDistance(24);
        robot.turnAbsolute(180);
        robot.driveDistance(24);
        robot.turnAbsolute(270);
        robot.driveDistance(24);
        robot.turnAbsolute(0);
    }
}
