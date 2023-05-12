package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(group = "drive")
public class ArmPIDTest extends LinearOpMode {

    public static double x = 10;
    public static double y = 400;

    private Arm arm;
    private ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        time.reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        arm = new Arm(hardwareMap, telemetry, time);

        waitForStart();
        while(!isStopRequested() && opModeIsActive()){
//
//            double normalizedY = (gamepad1.left_stick_y+1.0)/2.0;
//            arm.shoulderMotor.setTargetPosition((int) (normalizedY/1700.0));
//            arm.shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //2500 estimated elbow encoder ticks
            arm.elbowMotor.setPower(gamepad2.left_stick_y);
            arm.shoulderMotor.setPower(gamepad2.right_stick_y);
            arm.shoulderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            arm.elbowMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addData("arm1", arm.elbowMotor.getCurrentPosition());
            telemetry.addData("arm0", arm.shoulderMotor.getCurrentPosition()); //1700 encoder ticks for full range of motion, rotating from rear to front of the robot makes encoder go negative
//            arm.moveTo(new Vector2(x, y));
            telemetry.update();
        }

    }
}
