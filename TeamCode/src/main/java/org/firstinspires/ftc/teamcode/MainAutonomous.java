package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Set;

public class MainAutonomous extends LinearOpMode {
    Robot robot = new Robot(this);

    public void runOpMode() {
        robot.init();
        waitForStart();

        // TODO - add autonomous code here

        // close claw onto specimen
        robot.driveDistance(2*24 + 12);
    }
}
