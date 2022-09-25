package org.firstinspires.ftc.teamcode.alexstuff;
import java.lang.Math;

// input x and y coordinate
// output sth i'll do this later

public class ArmKinematics {
    public static void main(String[] args) {
        double x = 0.5; // in SI units
        double y = 0.5;

        double link = 0.5; // link length
        double link2 = 0.4;
        double[] angles = new double[2];

        angles[1] = Math
                .acos((Math.pow(x, 2) + Math.pow(y, 2) - Math.pow(link, 2) - Math.pow(link2, 2))
                        / (2 * link * link2));
        angles[0] = Math.atan(y / x)
                - Math.atan(link2 * Math.sin(angles[1]) / (link + link2 * Math.cos(angles[1])));
        if (x <= 0) {
            angles[1] *= -1;
            angles[0] *= Math.atan(y / x) + Math.atan(link2 * Math.sin(angles[1]) / (link + link2 * Math.cos(angles[1])));

        angles[0] /= 8; // conversion to ticks
        angles[1] /= 8;
        }
    }
}