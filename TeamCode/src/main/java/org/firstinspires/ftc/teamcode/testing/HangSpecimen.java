package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "Hang Specimen", group = "Testing")
public class HangSpecimen extends LinearOpMode {
    Robot robot = new Robot(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init();

        robot.claw.open();
        sleep(2000);
        robot.claw.close();

        robot.imu.resetYaw();

        robot.claw.start();
        robot.wrist.start();
        robot.bucket.start();
        robot.linearSlide.start();

       waitForStart();
       Actions.runBlocking(
               new SequentialAction(
                       robot.linearSlide.raiseLinearSlideToHangSpecimen(),
                       new SleepAction(2),
                       robot.hangSpecimen(),
                       new SleepAction(2)
               )
       );
    }
}
