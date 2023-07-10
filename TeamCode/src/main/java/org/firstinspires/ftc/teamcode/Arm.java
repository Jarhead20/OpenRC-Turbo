package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;

@Config
public class Arm {

    public static double p1 = 0.002, i1 = 0, d1 = 0, f1 = 0.05; //shoulder
    public static double p2 = 0.002, i2 = 0, d2 = 0, f2 = 0.03; //elbow

    // things for PIDF controller
    private PIDFController shoulderController;
    private PIDFController elbowController;

    public DcMotorEx shoulderMotor;
    public DcMotorEx elbowMotor;
    public Servo gripper;
    private Servo servo1;
    private Servo servo2;
    private double armX = 0;
    private double armY = 0;
    private double targetX = 0;
    private double targetY = 0;
    private double tolerance = 50;

    public Vector2[] armPoses = new Vector2[]{
            new Vector2(-200, 750), // dpad up - high pole
            new Vector2(350, 200), // dpad down - intake
            new Vector2(-300, 400), // dpad left - medium pole
            new Vector2(-300, 250), // dpad right - low pole
            new Vector2(100, 500), // 5 - high pole intermediate position
    };

    public int index = 0;
    public int position = 0;
    double prevX = armPoses[index].x;
    double prevY = armPoses[index].y;
    private double targetShoulderAngle = 0;
    private double targetElbowAngle = 0;
    ArmModel model = new ArmModel();

    Telemetry telemetry;
    ElapsedTime runtime;

    public Arm (HardwareMap map, Telemetry telemetry, ElapsedTime runtime) {
        shoulderController = new PIDFController(p1, i1, d1, 0, telemetry);
        elbowController = new PIDFController(p2, i2, d2, 0, telemetry);

        this.runtime = runtime;
        this.telemetry = telemetry;
        shoulderMotor = map.get(DcMotorEx.class, "arm0");
        elbowMotor = map.get(DcMotorEx.class, "arm1");
        gripper = map.get(Servo.class, "gripper");

        servo1 = map.get(Servo.class, "Servo2");
        servo2 = map.get(Servo.class, "Servo1");

        shoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shoulderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        shoulderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoulderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        elbowMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        elbowMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        servoMixer(-180, -180);
    }

    public void resetEncoder(){
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void armPIDF(int targetShoulderTicks, int targetElbowTicks) {
        shoulderController.setPIDF(p1, i1, d1, 0.0);
        elbowController.setPIDF(p2, i2, d2, 0.0);

        double shoulderPower = shoulderCalculatePIDF(shoulderMotor, elbowMotor, targetShoulderTicks);
        double elbowPower = elbowCalculatePIDF(elbowMotor, targetElbowTicks);
        telemetry.addData("elbowCurrentPower", elbowMotor.getPower());
        telemetry.addData("shoulderCurrentPower", shoulderMotor.getPower());
        telemetry.addData("shoulderAngle", model.encoderToDegrees(shoulderMotor.getCurrentPosition(), false)+model.LOWERARMSTARTANGLE);
        telemetry.addData("elbowAngle", model.encoderToDegrees(elbowMotor.getCurrentPosition(), true)+model.UPPERARMSTARTANGLE);
        telemetry.addData("shoulderPower", shoulderPower);
        telemetry.addData("elbowPower", elbowPower);
        telemetry.addData("shoulderCurrentTicks", shoulderMotor.getCurrentPosition());
        telemetry.addData("elbowCurrentTicks", elbowMotor.getCurrentPosition());
        telemetry.addData("shoulderTargetTicks", targetShoulderTicks);
        telemetry.addData("elbowTargetTicks", targetElbowTicks);


        shoulderMotor.setPower(shoulderPower);
        elbowMotor.setPower(elbowPower);
    }
    private double shoulderCalculatePIDF(DcMotorEx shoulder, DcMotorEx elbow, int targetShoulderTicks) {
        double motorPID = shoulderController.calculate(shoulder.getCurrentPosition(), targetShoulderTicks);
        double forwardFeed = Math.cos(Math.toRadians(model.encoderToDegrees(shoulder.getCurrentPosition(), false)+model.LOWERARMSTARTANGLE));
        forwardFeed += Math.cos(Math.toRadians(model.encoderToDegrees(elbow.getCurrentPosition(), true)+model.UPPERARMSTARTANGLE))/2.0;
        telemetry.addData("shoulderff", forwardFeed);
        telemetry.addData("shoulderpid", motorPID);
        return (forwardFeed * f1) + motorPID;
    }

    private double elbowCalculatePIDF(DcMotorEx motor, int targetEncoder) {
        double motorPID = elbowController.calculate(motor.getCurrentPosition(), targetEncoder);
        double forwardFeed = Math.cos(Math.toRadians(model.encoderToDegrees(motor.getCurrentPosition(), true)+model.UPPERARMSTARTANGLE));
        telemetry.addData("elbowff", forwardFeed);
        telemetry.addData("elbowpid", motorPID);
        return (forwardFeed * f2) + motorPID;
    }
    public void initLoop() throws InterruptedException {
        reportCurrentPosition();
    }

    boolean up = false;

    ElapsedTime armTimer = new ElapsedTime();

    public void inputGamepad(Gamepad gamepad){
        if (gamepad.b) openGripper();
        if (gamepad.a) closeGripper();
        if(gamepad.right_bumper) { //reset all arm positions
            for (Vector2 armPose : armPoses) armPose.reset();
        }

        if (gamepad.dpad_up){ // high pole
            position = 0;
            armTimer.reset();
        } else if (gamepad.dpad_down){ // intake
            position = 1;
        } else if (gamepad.dpad_left){ // medium pole
            position = 2;
        } else if (gamepad.dpad_right){ // low pole
            position = 3;
        }
        //add delay if going to a pole
        switch (position){
            case 0:
            case 1:
            case 2:
                if(armTimer.milliseconds() < 1000)
                    index = 4;
                else index = position;
                break;
            default:
                index = position;
                break;
        }

        telemetry.addData("index", index);

        Vector2 vec = armPoses[index];
        vec.x -= gamepad.left_stick_y*10;
        vec.y -= gamepad.right_stick_y*10;

        if(!moveTo(vec, true)){
//        if(!moveTo(vec, runtime.milliseconds() < 5000)){
            vec.x = prevX;
            vec.y = prevY;
        }
        prevX = vec.x;
        prevY = vec.y;
        telemetry.addData("armx", vec.x);
        telemetry.addData("army", vec.y);

        //Forward Kinematics
        reportCurrentPosition();
    }

    public boolean moveTo(Vector2 vec){
        return moveTo(vec, false);
    }
    public boolean moveTo(Vector2 vec, boolean wristOverride){
        //TODO: Add Range of Motion Constraints

        double[] angles = model.calculateMotorPositions((int)vec.x, (int)vec.y);
        targetX = vec.x;
        targetY = vec.y;
        if (angles == null){
            telemetry.addData("null", "null");
            return false;
        }

        targetShoulderAngle = angles[0];
        targetElbowAngle = angles[1];
        double roll = (angles[3]+1.0)*180.0-180.0;
        double pitch = (Math.toDegrees(angles[2]));
        telemetry.addData("pitch", pitch);
        pitch = Range.clip(pitch, -180, 180);
        if(!wristOverride)
            servoMixer(roll, pitch);

        telemetry.addData("targetShoulder", targetShoulderAngle);
        telemetry.addData("targetElbow", targetElbowAngle);
        telemetry.addData("currentShoulder", shoulderMotor.getCurrentPosition());
        telemetry.addData("currentElbow", elbowMotor.getCurrentPosition());

        shoulderMotor.setTargetPosition((int)targetShoulderAngle);
        elbowMotor.setTargetPosition((int)targetElbowAngle);
        setPower(0.5,0.5);
        shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        armPIDF((int) targetShoulderAngle, (int) targetElbowAngle);
        telemetry.addData("Pitch", angles[2]);
        return true;
    }

    public void servoMixer(double roll, double pitch){
        double servo1Position = (roll + pitch) / 2;
        double servo2Position = (roll - pitch) / 2;
        servo1Position = Range.clip((servo1Position + 180.0) / 360.0, -1, 1);
        servo2Position = Range.clip((servo2Position + 180.0) / 360.0, -1, 1);

        servo1.setPosition(servo1Position);
        servo2.setPosition(servo2Position);
    }

    public void reportCurrentPosition(){
        int shoulderRot = shoulderMotor.getCurrentPosition();
        int elbowRot = elbowMotor.getCurrentPosition();
        int[] armCoords = model.anglesToPosition(model.encoderToDegrees(shoulderRot, false) + model.LOWERARMSTARTANGLE, model.encoderToDegrees(elbowRot, true)+model.UPPERARMSTARTANGLE);
        armX = armCoords[0];
        armY = armCoords[1];
        telemetry.addData("shoulderRot", shoulderMotor.getCurrentPosition());
        telemetry.addData("elbowRot", elbowMotor.getCurrentPosition());

        telemetry.addData("armX", armX);
        telemetry.addData("armY", armY);
        telemetry.addData("targetX", armPoses[index].toString());
    }

    public void closeGripper(){
        gripper.setPosition(0.95);
    }

    public void openGripper(){
        gripper.setPosition(0.6);
    }

    public void toggleGripper(){
        gripper.setPosition(gripper.getPosition() > 0.9 ? 0 : 1.0);
    }

    public void setPower(double shoulder, double elbow){
        elbowMotor.setPower(elbow);
        shoulderMotor.setPower(shoulder);
    }

    public boolean atTarget(Vector2 vec){
        double[] angles = model.calculateMotorPositions((int)vec.x, (int)vec.y);
        targetShoulderAngle = angles[1];
        targetElbowAngle = -angles[0];
        if(Math.abs(shoulderMotor.getCurrentPosition() - targetShoulderAngle) < tolerance){
            if(Math.abs(elbowMotor.getCurrentPosition() - (-targetElbowAngle)) < tolerance) return true;
        }
        return false;
    }

    public boolean atTarget(Vector2 vec, double tol){
        double[] angles = model.calculateMotorPositions((int)vec.x, (int)vec.y);
        targetShoulderAngle = angles[1];
        targetElbowAngle = -angles[0];
        if(Math.abs(shoulderMotor.getCurrentPosition() - targetShoulderAngle) < tol){
            if(Math.abs(elbowMotor.getCurrentPosition() - (-targetElbowAngle)) < tol) return true;
        }
        return false;
    }
}