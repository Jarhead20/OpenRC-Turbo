package com.example.meepmeeptesting;

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
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-0.5,-61.0, Math.toRadians(0.0)))
                                .splineToSplineHeading(new Pose2d(26.875,-51.25,Math.toRadians(19.604088150829426)),Math.toRadians(19.604088150829426))
                                .splineToSplineHeading(new Pose2d(34.125,-24.5,Math.toRadians(74.83553910026156)),Math.toRadians(74.83553910026156))
                                .splineToSplineHeading(new Pose2d(4.25,-5.625,Math.toRadians(147.71533698889476)),Math.toRadians(147.71533698889476))
                                .splineToSplineHeading(new Pose2d(-34.375,2.0,Math.toRadians(168.83277153852663)),Math.toRadians(168.83277153852663))
                                .splineToSplineHeading(new Pose2d(-49.625,-22.5,Math.toRadians(238.0998430822663)),Math.toRadians(238.0998430822663))
                                .splineToSplineHeading(new Pose2d(-43.625,-42.25,Math.toRadians(-73.10135130596177)),Math.toRadians(-73.10135130596177))
                                .splineToSplineHeading(new Pose2d(-22.75,-48.0,Math.toRadians(-15.400170999529408)),Math.toRadians(-15.400170999529408))
                                .splineToSplineHeading(new Pose2d(-3.375,-34.5,Math.toRadians(34.86778843419524)),Math.toRadians(34.86778843419524))
                                .splineToSplineHeading(new Pose2d(5.25,-17.875,Math.toRadians(62.57985142665528)),Math.toRadians(62.57985142665528))
                                .splineToSplineHeading(new Pose2d(-23.875,-12.0,Math.toRadians(168.595520120853)),Math.toRadians(168.595520120853))
                                .splineToSplineHeading(new Pose2d(-11.625,12.625,Math.toRadians(63.55140349896741)),Math.toRadians(63.55140349896741))
                                .splineToSplineHeading(new Pose2d(15.875,17.75,Math.toRadians(10.556744755718213)),Math.toRadians(10.556744755718213))
                                .splineToSplineHeading(new Pose2d(37.625,31.125,Math.toRadians(31.589126435617537)),Math.toRadians(31.589126435617537))
                                .splineToSplineHeading(new Pose2d(-3.25,47.5,Math.toRadians(158.1683871534975)),Math.toRadians(158.1683871534975))
                                .splineToSplineHeading(new Pose2d(-34.125,37.875,Math.toRadians(197.3143918691125)),Math.toRadians(197.3143918691125))
                                .splineToSplineHeading(new Pose2d(-57.0,22.625,Math.toRadians(213.6900675259798)),Math.toRadians(213.6900675259798))
                                .splineToSplineHeading(new Pose2d(-58.625,4.875,Math.toRadians(264.7691928244468)),Math.toRadians(264.7691928244468))
                                .build()

                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}