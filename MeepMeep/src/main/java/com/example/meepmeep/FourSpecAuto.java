package com.example.meepmeep;

import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Arrays;
import java.util.Vector;

public class FourSpecAuto {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(1000);

        MinVelocityConstraint forcedSlowVelocity = new MinVelocityConstraint(Arrays.asList(
                new TranslationalVelocityConstraint(40.0),
                new AngularVelocityConstraint(Math.PI / 4)
        ));

        Pose2d initialPose = new Pose2d(0, -72 + 15.0 / 2, Math.toRadians(270));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(18, 18)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(initialPose)
                        // go to bar to deposit first spe
                        .strafeTo(new Vector2d(0, -34))

                        // deposit first spec


                        // push leftmost spec to observation zone
                        .setReversed(false)
                        .splineToConstantHeading(new Vector2d(37, -40), Math.toRadians(90))
                        .strafeTo(new Vector2d(37, -10))
                        .splineToSplineHeading(new Pose2d(42.75, -2, Math.toRadians(0)), Math.toRadians(0), new AngularVelocityConstraint(Math.PI / 1.5), null)
                        .splineToSplineHeading(new Pose2d(48, -10, Math.toRadians(90)), Math.toRadians(270), new AngularVelocityConstraint(Math.PI / 1.5), null)
                        .splineToConstantHeading(new Vector2d(46, -57), Math.toRadians(270))

//                        // push middle spec to observation zone
//                        .strafeTo(new Vector2d(46, -15))
//                        .splineToConstantHeading(new Vector2d(49.5, -8), Math.toRadians(0), new TranslationalVelConstraint(40.0))
//                        .splineToConstantHeading(new Vector2d(53, -15), Math.toRadians(270), new TranslationalVelConstraint(40.0))
//                        .strafeTo(new Vector2d(53, -54))

                        // pickup spec #2
                        .setTangent(Math.toRadians(125))
                        .splineToConstantHeading(new Vector2d(38, -72 + 3), Math.toRadians(280), new TranslationalVelocityConstraint(40.0), null)

                        // hang spec #2
                        .setTangent(Math.toRadians(135))
                        .splineToSplineHeading(new Pose2d(-5, -45, Math.toRadians(-90)), Math.toRadians(90))
                        .strafeTo(new Vector2d(-5, -34), forcedSlowVelocity, null)


                        // pickup spec #3
                        .setReversed(false)
                        .splineToSplineHeading(new Pose2d(40, -72 + 15, Math.toRadians(90)), Math.toRadians(270))
                        .strafeTo(new Vector2d(40, -72 + 3), forcedSlowVelocity, null)

                        // hang spec #3
                        .setTangent(Math.toRadians(135))
                        .splineToSplineHeading(new Pose2d(5, -45, Math.toRadians(-90)), Math.toRadians(90))
                        .strafeTo(new Vector2d(5, -34), forcedSlowVelocity, null)


//                        // pickup spec #4
//                        .splineToSplineHeading(new Pose2d(40, -72 + 9, Math.toRadians(90)), Math.toRadians(270))
//                        .stopAndAdd(robot.pickupSpecimen())
//                        .afterTime(0.2, robot.linearSlide.raiseLinearSlideToHangSpecimen())
//
//                        // hang spec #4
//                        .setReversed(false)
//                        .splineToSplineHeading(new Pose2d(-1.5, -35, Math.toRadians(270)), Math.toRadians(90))
//                        .stopAndAdd(robot.hangSpecimen())
//
//                        // park
//                        .strafeTo(new Vector2d(60, -55))
                        .build());

        meepMeep.setBackground(org.rowlandhall.meepmeep.MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
