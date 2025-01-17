package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Settings;

@Autonomous(name="2 Specimen Risky", group="Main", preselectTeleOp="Main Teleop")
public class TwoSpecimenRisk extends LinearOpMode {
    Robot robot = new Robot(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init();

        // close claw for specimen
        robot.claw.open();
        sleep(1000);
        robot.claw.close();

        waitForStart();

        // move linear slide to above bar
        robot.linearSlide.moveToPosition(Settings.LinearSlide.SPECIMEN_APPROACH_POSITION, 1);
        sleep(150);

        // drive to specimen hanging bar
        robot.driveDistance(-29, 2000);

        // release specimen #1
        robot.linearSlide.moveToPosition(Settings.LinearSlide.SPECIMEN_LOWERED_POSITION, 0.5);
        sleep(1000);
        robot.linearSlide.stop();
        robot.claw.open();
        sleep(1500);

        // lower linear slide to starting position
        robot.linearSlide.moveToPosition(Settings.LinearSlide.STARTING_POSITION, 1);

        // go to wall to get to specimen #2
        robot.driveDistance(15, 1000);
        robot.strafeDistance(-45, 2000);

        // turn around to face specimen #2
        robot.turnAbsolute(180, 1000);

        // drive to specimen #2
        robot.driveDistance(-14.5, 1500);

        // pick up specimen #2
        robot.claw.close();
        sleep(1500);
        robot.linearSlide.moveToPosition(100, 0.5);

        // drive back to specimen hanging bar
        robot.driveDistance(10.25, 800);
        robot.strafeDistance(-50, 2000);

        // turn to face specimen hanging bar
        robot.turnAbsolute(0, 1000);

        // raise linear slide to above bar
        robot.linearSlide.moveToPosition(Settings.LinearSlide.SPECIMEN_APPROACH_POSITION + 250, 1);
        sleep(500);

        // drive into specimen hanging bar
        robot.driveDistance(-20, 2000);

        // release specimen #2
        robot.linearSlide.moveToPosition(Settings.LinearSlide.SPECIMEN_LOWERED_POSITION, 0.5);
        sleep(1000);
        robot.linearSlide.stop();
        robot.claw.open();
        sleep(1500);

        // lower linear slide to starting position
        robot.linearSlide.moveToPosition(Settings.LinearSlide.STARTING_POSITION, 1);

        // BOOK IT TO THE CORNER!!!
        robot.driveDistance(23, 1000);
        robot.driveBase.setMotorPowers(1, -1, -1, 1);
    }
}
