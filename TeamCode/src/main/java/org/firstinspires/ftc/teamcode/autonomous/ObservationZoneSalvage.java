package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name="Observation Zone Salvage", group="Extra", preselectTeleOp="Main Teleop")
public class ObservationZoneSalvage extends LinearOpMode {
    Robot robot = new Robot(this);

    public void runOpMode() {
        robot.init();
        waitForStart();

        // drives to the right for observation zone points
        robot.driveBase.setMotorPowers(0.5, -0.5, -0.5, 0.5);
        sleep(1000);
        robot.driveBase.stop();
    }
}
