package org.firstinspires.ftc.teamcode;
import java.lang.Math;

public class ArmKinematics {
    public static void main(String[] args) {
        double x = 0.5; // in SI units
        double y = 0.5;

        double link = 0.5; // link length
        double link2 = 0.4;
        double[] output = new double[4]; // pos q2 --> q1, neg q2 --> q1

        output[0] = Math.acos((x*x + y*y - link*link - link2*link2)/(2*link*link2));
        double beta = Math.atan((link2*Math.sin(output[0]))/(link + link2*Math.cos(output[0])));
        double gamma = Math.atan(y/x);
        output[1] = gamma - beta;
        output[2] = -output[0]; // two solutions due to use of cosine rule
        output[3] = gamma + beta;

        System.out.println("Angle of 2nd link:" + output[0]);
        System.out.println("Angle of 1st link:" + output[1]);
        System.out.println("Angle of 2nd link, 2nd solution:" + output[2]);
        System.out.println("Angle of 2nd link, 2nd solution:" + output[3]);

        // on collection side use 1st solution
        // drop side use 2nd solution
    }
}