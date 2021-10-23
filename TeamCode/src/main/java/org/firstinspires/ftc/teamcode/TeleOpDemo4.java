package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
//192.168.43.1:8080/dash
@TeleOp(name="Mecanum test 4", group="Iterative Opmode")
public class TeleOpDemo4 extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor duck = null;
    private DcMotor intake = null;
//    private DcMotor vroom = null;
//    private DcMotor arm1 = null;
//    private DcMotor arm2 = null;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        //TODO: uncomment this too
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "leftBack");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");
        duck = hardwareMap.get(DcMotor.class, "duck");
        intake = hardwareMap.get(DcMotor.class, "intake");
//        vroom = hardwareMap.get(DcMotor.class, "vroom");
//        arm1 = hardwareMap.get(DcMotor.class, "arm1");
//        arm2 = hardwareMap.get(DcMotor.class, "arm2");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        duck.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);

//        vroom.setDirection(DcMotor.Direction.FORWARD);
//        arm1.setDirection(DcMotor.Direction.FORWARD);
//        arm2.setDirection(DcMotor.Direction.FORWARD);
//        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);




        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
//        arm1.setPower(1);
//        arm2.setPower(1);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
//        double leftPower;
//        double rightPower;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
//        vroom.setPower(1);

//        if(gamepad2.a){
//            arm1.setTargetPosition(arm1.getTargetPosition()+10);
//            arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
//        else if (gamepad2.b){
//            arm1.setTargetPosition(arm1.getTargetPosition()-10);
//            arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
//        if(gamepad2.x){
//            arm2.setTargetPosition(arm2.getTargetPosition()+1);
//            arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
//        else if (gamepad2.y){
//            arm2.setTargetPosition(arm2.getTargetPosition()-1);
//            arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }

        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_y;
        double[] v = new double[4];
        v[0] = r * Math.cos(robotAngle) + rightX;
        v[1] = r * Math.sin(robotAngle) - rightX;
        v[2] = r * Math.sin(robotAngle) + rightX;
        v[3] = r * Math.cos(robotAngle) - rightX;
        double max = 0;
        for (int i = 0; i < 4; i++) {
            if(Math.abs(v[i]) > max) max = Math.abs(v[i]);
        }
        for (int i = 0; i < 4; i++) {
            double e = v[i] / (v[i]);
            if(v[i] < 0) v[i] = -e;
            else v[i] = e;
        }


        leftFrontDrive.setPower(v[0]);
        rightFrontDrive.setPower(v[1]);
        leftBackDrive.setPower(v[2]);
        rightBackDrive.setPower(v[3]);
        if(gamepad1.a) duck.setPower(1);
        else duck.setPower(0);
        if(gamepad1.b) intake.setPower(1);
        else intake.setPower(0);


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
//        telemetry.addData("Motors", "arm1 " + arm1.getCurrentPosition() + " " + arm1.getTargetPosition());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}





//Josh was here lol
