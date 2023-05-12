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

    public static double p1 = 0, i1 = 0, d1 = 0, f1 = 3; //shoulder
    public static double p2 = 0, i2 = 0, d2 = 0, f2 = -5.0; //elbow

    // things for PIDF controller
    private PIDFController shoulderController;
    private PIDFController elbowController;



    public DcMotorEx shoulderMotor;
    public DcMotorEx elbowMotor;
    public Servo gripper;
    private Servo pitch;
    private Servo roll;
    private double armX = 0;
    private double armY = 0;
    private double targetX = 0;
    private double targetY = 0;
    private double tolerance = 50;
    private Vector2[] armPoses = new Vector2[]{
            new Vector2(405, 405),
            new Vector2(-350, 30),
            new Vector2(-480, 30),
            new Vector2(200, 300),
            new Vector2(-430, 200),
            new Vector2(-10, 700),
            new Vector2(-450, 500),
            new Vector2(-350, 200),
    };

    public int index = 0;
    public int position = 0;

    private double targetShoulderAngle = 0; // used in PIDF, should it go up?
    private double targetElbowAngle = 0;
    ArmModel model = new ArmModel();

    Telemetry telemetry;
    ElapsedTime runtime;

    public Arm (HardwareMap map, Telemetry telemetry, ElapsedTime runtime) {
        shoulderController = new PIDFController(p1, i1, d1, f1, telemetry);
        elbowController = new PIDFController(p2, i2, d2, f2, telemetry);

        this.runtime = runtime;
        this.telemetry = telemetry;
        shoulderMotor = map.get(DcMotorEx.class, "arm0");
        elbowMotor = map.get(DcMotorEx.class, "arm1");
        gripper = map.get(Servo.class, "gripper");

        pitch = map.get(Servo.class, "pitch");
        roll = map.get(Servo.class, "roll");
        roll.setDirection(Servo.Direction.REVERSE);

        shoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shoulderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        shoulderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoulderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        elbowMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        elbowMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void armPIDF(int targetShoulderTicks, int targetElbowTicks) {
        shoulderController.setPIDF(p1, i1, d1, f1);
        elbowController.setPIDF(p2, i2, d2, f2);

        double shoulderPower = shoulderCalculatePIDF(shoulderMotor, elbowMotor, targetShoulderTicks, targetElbowTicks);
        double elbowPower = elbowCalculatePIDF(elbowMotor, targetElbowTicks);
        telemetry.addData("elbowCurrentPower", elbowMotor.getPower());
        telemetry.addData("shoulderCurrentPower", shoulderMotor.getPower());
        telemetry.addData("shoulderAngle", model.encoderToDegrees(shoulderMotor.getCurrentPosition(), false)+model.bicepStartAngle);
        telemetry.addData("elbowAngle", model.encoderToDegrees(elbowMotor.getCurrentPosition(), true)+model.forearmStartAngle);
        telemetry.addData("shoulderPower", shoulderPower);
        telemetry.addData("elbowPower", elbowPower);
        telemetry.addData("shoulderCurrentTicks", shoulderMotor.getCurrentPosition());
        telemetry.addData("elbowCurrentTicks", elbowMotor.getCurrentPosition());
        telemetry.addData("shoulderTargetTicks", targetShoulderTicks);
        telemetry.addData("elbowTargetTicks", targetElbowTicks);


        shoulderMotor.setPower(shoulderPower/100.0);
        elbowMotor.setPower(elbowPower/100.0);
    }
    private double shoulderCalculatePIDF(DcMotorEx shoulder, DcMotorEx elbow, int targetShoulderTicks, int targetElbowTicks) {
        double motorPID = shoulderController.calculate(shoulder.getCurrentPosition(), targetShoulderTicks);
        double forwardFeed = Math.cos(Math.toRadians(model.encoderToDegrees(shoulder.getCurrentPosition(), false)+model.bicepStartAngle));
        forwardFeed += Math.cos(Math.toRadians(model.encoderToDegrees(elbow.getCurrentPosition(), true)+model.forearmStartAngle))/2.0;
        telemetry.addData("shoulderff", forwardFeed);
        telemetry.addData("shoulderpid", motorPID);
        return (forwardFeed * f1);
    }

    private double elbowCalculatePIDF(DcMotorEx motor, int targetEncoder) {
        double motorPID = elbowController.calculate(motor.getCurrentPosition(), targetEncoder);
        double forwardFeed = Math.cos(Math.toRadians(model.encoderToDegrees(motor.getCurrentPosition(), true)+model.forearmStartAngle));
        telemetry.addData("elbowff", forwardFeed);
        telemetry.addData("elbowpid", motorPID);
        telemetry.addData("elbowpidcoeficcients", elbowController.getCoefficients()[0]);
        return (forwardFeed * f2);
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

//        elbowMotor.setPower(0.4);
//        shoulderMotor.setPower(0.6);

        if (gamepad.dpad_up){
            position = 0;
//            elbowMotor.setPower(0.8);
//            shoulderMotor.setPower(0.3);
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
        vec.x += gamepad.left_stick_y*4;
        vec.y -= gamepad.right_stick_y*4;
//            vec.x = Range.clip(vec.x, -900, -1);
        moveTo(vec);
        //Inverse Kinematics


        //Forward Kinematics
        reportCurrentPosition();
    }

    public void moveTo(Vector2 vec){
        //TODO: Add Range of Motion Constraints

        double[] angles = model.calculateMotorPositions((int)vec.x, (int)vec.y);
        targetX = vec.x;
        targetY = vec.y;
        if (angles == null){
            telemetry.addData("null", "null");
            return;
        }
        if (angles[1] < 0){
            telemetry.addLine("less than zero");
//            return;
        }

        targetShoulderAngle = angles[1];
        targetElbowAngle = -angles[0];

        shoulderMotor.setTargetPosition((int)targetShoulderAngle);
        elbowMotor.setTargetPosition((int)-targetElbowAngle);
        telemetry.addData("t1", targetShoulderAngle);
        telemetry.addData("t2", targetElbowAngle);
        telemetry.addData("c1", shoulderMotor.getCurrentPosition());
        telemetry.addData("c2", elbowMotor.getCurrentPosition());
        shoulderMotor.setPower(1.0);
        elbowMotor.setPower(1.0);
        shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int shoulderX = (int) (405 * Math.cos(Math.toRadians(model.encoderToDegrees(shoulderMotor.getCurrentPosition(), false) + + model.bicepStartAngle)));
        int shoulderY = (int) (405 * Math.sin(Math.toRadians(model.encoderToDegrees(shoulderMotor.getCurrentPosition(), false) + + model.bicepStartAngle)));
        int elbowX = (int) (405 * Math.cos(Math.toRadians(model.encoderToDegrees(elbowMotor.getCurrentPosition(), true) + + model.forearmStartAngle)));
        int elbowY = (int) (405 * Math.sin(Math.toRadians(model.encoderToDegrees(elbowMotor.getCurrentPosition(), true) + + model.forearmStartAngle)));
        telemetry.addData("elbowX", elbowX);
        telemetry.addData("shoulderX", shoulderX);
        telemetry.addData("elbowY", elbowY);
        telemetry.addData("shoulderY", shoulderY);
        telemetry.addData("shoulderAngle", model.encoderToDegrees(shoulderMotor.getCurrentPosition(), false) + + model.bicepStartAngle);
//        armPIDF((int) targetShoulderAngle, (int) targetElbowAngle);
//        pitch.setPosition(1-angles[2]);
//        telemetry.addData("Pitch", angles[2]);
//        roll.setPosition(angles[3]);
    }

    public void reportCurrentPosition(){
        int shoulderRot = shoulderMotor.getCurrentPosition();
        int elbowRot = elbowMotor.getCurrentPosition();
        int[] armCoords = model.anglesToPosition(model.encoderToDegrees(shoulderRot, false) + model.bicepStartAngle, model.encoderToDegrees(elbowRot, true)+model.forearmStartAngle);
        armX = armCoords[0];
        armY = armCoords[1];
        telemetry.addData("shoulderRot", shoulderMotor.getCurrentPosition());
        telemetry.addData("elbowRot", elbowMotor.getCurrentPosition());

        telemetry.addData("armX", armX);
        telemetry.addData("armY", armY);
        telemetry.addData("targetX", armPoses[index].toString());
    }

    public void closeGripper(){
        gripper.setPosition(1.0);
    }

    public void openGripper(){
        gripper.setPosition(0);
    }

    public void toggleGripper(){
        gripper.setPosition(gripper.getPosition() > 0.9 ? 0 : 1.0);
    }

    public void goToServo(double x, double y, double x2, double y2){
        double gradient1 = (x-x2)/4;
        double gradient2 = (y-y2)/4;
        double newx = Range.clip((runtime.seconds()*gradient1)+x2, Math.min(x, x2), Math.max(x, x2));
        double newy = Range.clip((runtime.seconds()*gradient2)+y2, Math.min(y, y2), Math.max(y, y2));
        pitch.setPosition(newx);
        roll.setPosition(newy);
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