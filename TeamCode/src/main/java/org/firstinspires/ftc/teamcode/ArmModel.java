package org.firstinspires.ftc.teamcode;

public class ArmModel {
    //Distances all in mm
    private final double bicepLength = 400;
    private final double forearmLength = 420;

    //Information About motor
    private final int ticksPerRevolution = 1440;
    private final double gearRatio = (40.0/18);

    //Coordinates of the base
    private final int baseX = 0;
    private final int baseY = 0;

    //Start Angle in degrees
    private final int forearmStartAngle = 0;
    private final int bicepStartAngle = 135;

    public int radiansToEncoder(double radians){
        double revolutions = radians / (2 * Math.PI);
        int ticks = (int) (revolutions * ticksPerRevolution * gearRatio);

        return ticks;
    }
    public int[] calculateMotorPositions(int targetX, int targetY) {
        //Calculate distance to target
        double distance = Math.sqrt(Math.pow(targetX - baseX, 2) + Math.pow(targetY - baseY, 2));
        if(distance> bicepLength + forearmLength){
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
                wristPitch = (Math.PI - innerElbowAngle) / Math.PI;

            }
            else {
                wristRoll = 1;
                //Calculate lower motor angle
                lowerMotorAngle = hypotenuseAngle + Math.asin((Math.sin(innerElbowAngle) * forearmLength) / distance);
                //Calculate upper motor angle
                upperMotorAngle = Math.PI - (Math.PI - innerElbowAngle) - (Math.PI - lowerMotorAngle);
                //Calculate wrist pitch (0-1)
                wristPitch = (Math.PI - innerElbowAngle) / Math.PI;
            }
            System.out.println("Upper motor angle" + Math.toDegrees(upperMotorAngle));
            System.out.println("Lower motor angle" + Math.toDegrees(lowerMotorAngle));

            //calculate target encoder position
            //First convert to revolutions
            int upperMotorPosition = radiansToEncoder(upperMotorAngle);
            int lowerMotorPosition = radiansToEncoder(lowerMotorAngle);

            //Account for start angle
            upperMotorPosition -= radiansToEncoder(Math.toRadians(bicepStartAngle));
            lowerMotorPosition -= radiansToEncoder(Math.toRadians(forearmStartAngle));

            return new int[]{upperMotorPosition, lowerMotorPosition, (int) wristPitch, (int) wristRoll};
        }
    }
}
