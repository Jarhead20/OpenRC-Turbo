package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 8)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35.77, 2.64, Math.toRadians(88.07)))
                                .splineToSplineHeading(new Pose2d(-37.97, 19.92, Math.toRadians(92.68)), Math.toRadians(89.10))
                                .splineToSplineHeading(new Pose2d(33.97, 34.10, Math.toRadians(94.33)), Math.toRadians(2.66))
                                .splineTo(new Vector2d(33.20, 54.60), Math.toRadians(79.48))



                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}