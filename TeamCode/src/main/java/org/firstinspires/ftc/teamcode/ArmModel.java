package org.firstinspires.ftc.teamcode;

public class ArmModel {
    //Distances all in mm
    private final double bicepLength = 420;
    private final double forearmLength = 430;

    //Information About motor
    private final int ticksPerRevolution = 1440;
    private final double gearRatio = (40.0/18.0);

    //Coordinates of the base
    private final int baseX = 0;
    private final int baseY = 0;

    //Start Angle in degrees
    private final int forearmStartAngle = 180;
    private final int bicepStartAngle = 45;

    //Joint Limits
    private final int minInnerElbowAngle = 30;

    public int radiansToEncoder(double radians){
        double revolutions = radians / (2 * Math.PI);
        int ticks = (int) (revolutions * ticksPerRevolution * gearRatio);

        return ticks;
    }
    public double[] calculateMotorPositions(int targetX, int targetY) {
        //Calculate distance to target
        double distance = Math.sqrt(Math.pow(targetX - baseX, 2) + Math.pow(targetY - baseY, 2));
        if(distance> (bicepLength + forearmLength)){
            return null;
        }
        else{
            //Calculate Angle of Hypotenuse
            double hypotenuseAngle = Math.atan2(targetY - baseY, targetX - baseX);
            //Calculate angle of elbow using law of cosines
            double innerElbowAngle = Math.acos((Math.pow(bicepLength, 2) + Math.pow(forearmLength, 2) - Math.pow(distance, 2)) / (2 * bicepLength * forearmLength));
            double lowerMotorAngle;
            double upperMotorAngle;
            double wristRoll;
            double wristPitch;

            //Find the motor angles (if statement to make sure the arm goes overarm)
            if (targetX < baseX) {
                wristRoll = 0;
                //Calculate lower motor angle
                lowerMotorAngle = hypotenuseAngle - Math.asin((Math.sin(innerElbowAngle) * forearmLength) / distance);
                //Calculate upper motor angle
                upperMotorAngle = Math.PI - innerElbowAngle + lowerMotorAngle;
                //Calculate wrist pitch (0-1)
                wristPitch = map(upperMotorAngle, Math.PI/2, 3*Math.PI/2, 0, 1);

            }
            else {
                wristRoll = 1;
                //Calculate lower motor angle
                lowerMotorAngle = hypotenuseAngle + Math.asin((Math.sin(innerElbowAngle) * forearmLength) / distance);
                //Calculate upper motor angle
                upperMotorAngle = Math.PI - (Math.PI - innerElbowAngle) - (Math.PI - lowerMotorAngle);
                //Calculate wrist pitch (0-1)
                wristPitch = map(upperMotorAngle, 3*Math.PI/2, Math.PI/2, 0, 1);
            }

            //calculate target encoder position
            //First convert to revolutions
            upperMotorAngle -= Math.toRadians(forearmStartAngle);
            lowerMotorAngle -= Math.toRadians(bicepStartAngle);

            int upperMotorPosition = radiansToEncoder(upperMotorAngle);
            int lowerMotorPosition = radiansToEncoder(lowerMotorAngle);

            return new double[]{upperMotorPosition, lowerMotorPosition,  wristPitch, wristRoll};
        }
    }

    public float encoderToDegrees(int encoder){
        return (float) (encoder * (360.0 / ticksPerRevolution) / gearRatio);
    }
    public double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
    public int[] anglesToPosition(float ShoulderRot, float ElbowRot){
        //Using forward-kinematics, the position can be calculated
        int elbowX = (int) (bicepLength * Math.cos(Math.toRadians(ShoulderRot)));
        int elbowY = (int) (bicepLength * Math.sin(Math.toRadians(ShoulderRot)));
        int wristX = (int) (elbowX + forearmLength * Math.cos(Math.toRadians(ElbowRot)));
        int wristY = (int) (elbowY + forearmLength * Math.sin(Math.toRadians(ElbowRot)));
        return new int[]{wristX, wristY};
    }
}
