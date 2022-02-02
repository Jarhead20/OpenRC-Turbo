package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;

public class MeepMeepTesting {
    public static void main(String[] args) {
        // TODO: If you experience poor performance, enable this flag
         System.setProperty("sun.java2d.opengl", "true");

        // Declare a MeepMeep instance
        // With a field size of 800 pixels
        MeepMeep mm = new MeepMeep(900)
                // Set field image
                .setBackground(MeepMeep.Background.FIELD_FREIGHT_FRENZY)
                // Set theme
                .setBotDimensions(12,16)
                .setTheme(new ColorSchemeRedDark())
                // Background opacity from 0-1
                .setBackgroundAlpha(1f)
                // Set constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(20, 20, Math.toRadians(180), Math.toRadians(180), 10)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(10, -65, Math.toRadians(270)))
                                .setReversed(true)
                                .addDisplacementMarker(() -> {
                                    System.out.println("extend slide");
                                })
                                .splineTo(new Vector2d(0,-40),Math.toRadians(120))

                                .setReversed(false)
                                .addDisplacementMarker(() -> {
                                    System.out.println("spin servo");
                                })
//                                .addDisplacementMarker()
//                                .splineToConstantHeading()
                                .splineTo(new Vector2d(23,-63),Math.toRadians(0))
                                .addDisplacementMarker(() -> {
                                    System.out.println("spin intake");
                                    System.out.println("retract slide");
                                })
                                .forward(30)
                                .addDisplacementMarker(() -> {
                                    System.out.println("stop intake");
                                })
                                .back(30)
                                .setReversed(true)
                                .addDisplacementMarker(() -> {
                                    System.out.println("extend slide");
                                })
                                .splineTo(new Vector2d(0,-40),Math.toRadians(120))

                                .setReversed(false)
                                .addDisplacementMarker(() -> {
                                    System.out.println("spin servo");
                                })
//                                .addDisplacementMarker()
//                                .splineToConstantHeading()
                                .splineTo(new Vector2d(23,-63),Math.toRadians(0))
                                .addDisplacementMarker(() -> {
                                    System.out.println("spin intake");
                                    System.out.println("retract slide");
                                })
                                .forward(30)
                                .addDisplacementMarker(() -> {
                                    System.out.println("stop intake");
                                })
                                .back(30)
                                .setReversed(true)
                                .addDisplacementMarker(() -> {
                                    System.out.println("extend slide");
                                })
                                .splineTo(new Vector2d(0,-40),Math.toRadians(120))

                                .setReversed(false)
                                .addDisplacementMarker(() -> {
                                    System.out.println("spin servo");
                                })
//                                .addDisplacementMarker()
//                                .splineToConstantHeading()
                                .splineTo(new Vector2d(23,-63),Math.toRadians(0))
                                .addDisplacementMarker(() -> {
                                    System.out.println("spin intake");
                                    System.out.println("retract slide");
                                })
                                .forward(30)
                                .addDisplacementMarker(() -> {
                                    System.out.println("stop intake");
                                })
                                .back(30)
                                .setReversed(true)
                                .addDisplacementMarker(() -> {
                                    System.out.println("extend slide");
                                })
                                .splineTo(new Vector2d(0,-40),Math.toRadians(120))

                                .setReversed(false)
                                .addDisplacementMarker(() -> {
                                    System.out.println("spin servo");
                                })
//                                .addDisplacementMarker()
//                                .splineToConstantHeading()
                                .splineTo(new Vector2d(23,-63),Math.toRadians(0))
                                .addDisplacementMarker(() -> {
                                    System.out.println("spin intake");
                                    System.out.println("retract slide");
                                })
                                .forward(30)
                                .addDisplacementMarker(() -> {
                                    System.out.println("stop intake");
                                })
                                .back(30)
                                .build()
                )
                .start();
    }
}