package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.text.DecimalFormat;

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
@TeleOp(name="Main2", group="Iterative Opmode")
public class Main extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private Drive drive;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        drive = new Drive(hardwareMap,telemetry);
        drive.setup();
        drive.getDuck().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
//        if(gamepad1.right_bumper) drive.setMultiplier(0.5);
//        else drive.setMultiplier(1);
        drive.setMultiplier(1-gamepad1.right_trigger);

//        drive.mecanum(gamepad1);

        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = -gamepad1.right_stick_x;
        double[] v = new double[4];
        v[0] = r * Math.cos(robotAngle) + rightX;
        v[1] = r * Math.sin(robotAngle) - rightX;
        v[2] = r * Math.sin(robotAngle) + rightX;
        v[3] = r * Math.cos(robotAngle) - rightX;
        double max = 0;
        for (int i = 0; i < 4; i++) {
            if(Math.abs(v[i]) > max) max = Math.abs(v[i]);
        }
        max = Range.clip(max,-1,1);
        DecimalFormat df = new DecimalFormat("0.00");
        telemetry.addData("Status", "max: " + max);
        for (int i = 0; i < 4; i++) {
            double e = v[i] / max;
            telemetry.addData("Status", df.format(e) + " " + df.format(v[i]) + " " + df.format(max));
            v[i] = e;
        }


        drive.getLeftFrontDrive().setPower(v[0]*drive.getMultiplier());
        drive.getRightFrontDrive().setPower(v[1]*drive.getMultiplier());
        drive.getLeftBackDrive().setPower(v[2]*drive.getMultiplier());
        drive.getRightBackDrive().setPower(v[3]*drive.getMultiplier());
//        telemetry.addData("Status", v[0]);
//        telemetry.addData("Status", v[1]);
//        telemetry.addData("Status", v[2]);
//        telemetry.addData("Status", v[3]);
        if(gamepad1.a) drive.getDuck().setPower(1);
        else drive.getDuck().setPower(0);

        if(gamepad1.b) drive.getIntake().setPower(1);
        else drive.getIntake().setPower(0);

//        telemetry.addData("Status", "Run Time: " + runtime.toString());
//        telemetry.addData("Status", gamepad1.a);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}