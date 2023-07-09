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
    private Vector2[] armPoses = new Vector2[]{
            new Vector2(300, 700),
            new Vector2(-350, 200),
            new Vector2(-350, 100),
            new Vector2(200, 300),
            new Vector2(-430, 200),
            new Vector2(-10, 700),
            new Vector2(-450, 500),
            new Vector2(-350, 200),
    };

    public int index = 0;
    public int position = 0;
    double prevX = armPoses[index].x;
    double prevY = armPoses[index].y;
    private double targetShoulderAngle = 0; // used in PIDF, should it go up?
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
//        servo2.setDirection(Servo.Direction.REVERSE);

        shoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shoulderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoulderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoulderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        elbowMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void armPIDF(int targetShoulderTicks, int targetElbowTicks) {
        shoulderController.setPIDF(p1, i1, d1, 0.0);
        elbowController.setPIDF(p2, i2, d2, 0.0);

        double shoulderPower = shoulderCalculatePIDF(shoulderMotor, elbowMotor, targetShoulderTicks, targetElbowTicks);
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
    private double shoulderCalculatePIDF(DcMotorEx shoulder, DcMotorEx elbow, int targetShoulderTicks, int targetElbowTicks) {
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

        if (gamepad.b){
            openGripper();
        }
        if (gamepad.a){
            closeGripper();
        }

        if (gamepad.dpad_up){
            position = 0;
            index = gamepad.left_bumper ? position : position+3;
            armTimer.reset();
        }

        else if (gamepad.dpad_down){
            position = 1;
            index = gamepad.left_bumper ? position : position+3;
        }

        else if (gamepad.dpad_left){
            position = 2;
            index = gamepad.left_bumper ? position : position+3;
        }

        switch (position){
            case 0:
                if(armTimer.milliseconds() < 500)
                    index = 6;
                else index = position;
                break;
            case 1:
                if(armTimer.milliseconds() < 1000)
                    index = 7;
                else index = position;
                break;
            case 2:
                index = position;
                break;
        }

        telemetry.addData("index", index);


        Vector2 vec = armPoses[index];
        vec.x += gamepad.left_stick_y*10;
        vec.y -= gamepad.right_stick_y*10;

        if(!moveTo(vec)){
            vec.x = prevX;
            vec.y = prevY;
        }
        prevX = vec.x;
        prevY = vec.y;
        telemetry.addData("armx", vec.x);
        telemetry.addData("army", vec.y);

//            vec.x = Range.clip(vec.x, -900, -1);
//        moveTo(vec);
        //Inverse Kinematics


        //Forward Kinematics
        reportCurrentPosition();
    }

    public boolean moveTo(Vector2 vec){
        //TODO: Add Range of Motion Constraints

        double[] angles = model.calculateMotorPositions((int)vec.x, (int)vec.y);
        targetX = vec.x;
        targetY = vec.y;
        if (angles == null){
            telemetry.addData("null", "null");
            return false;
        }
        if (vec.y <= 0){
            telemetry.addLine("less than zero");
            return false;
        }

        targetShoulderAngle = angles[0];
        targetElbowAngle = angles[1];
        double roll = (angles[3]+1.0)*180.0-180.0;
        double pitch = (Math.toDegrees(angles[2]));
        telemetry.addData("pitch", pitch);
        pitch = Range.clip(pitch, -180, 180);
        double servo1Position = (roll + pitch) / 2;
        double servo2Position = (roll - pitch) / 2;
        servo1Position = Range.clip((servo1Position + 180.0) / 360.0, -1, 1);
        servo2Position = Range.clip((servo2Position + 180.0) / 360.0, -1, 1);



        servo1.setPosition(servo1Position);
        servo2.setPosition(servo2Position);

        shoulderMotor.setTargetPosition((int)targetShoulderAngle);
        elbowMotor.setTargetPosition((int)targetElbowAngle);
        setPower(0.5,0.5);
        closeGripper();
//        shoulderMotor.setPower(1.0);
//        elbowMotor.setPower(1.0);
        shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);



//        armPIDF((int) targetShoulderAngle, (int) targetElbowAngle);
//        telemetry.addData("Pitch", angles[2]);
        return true;
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

    public void goToServo(double x, double y, double x2, double y2){
        double gradient1 = (x-x2)/4;
        double gradient2 = (y-y2)/4;
        double newx = Range.clip((runtime.seconds()*gradient1)+x2, Math.min(x, x2), Math.max(x, x2));
        double newy = Range.clip((runtime.seconds()*gradient2)+y2, Math.min(y, y2), Math.max(y, y2));
//        pitch.setPosition(newx);
//        roll.setPosition(newy);
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