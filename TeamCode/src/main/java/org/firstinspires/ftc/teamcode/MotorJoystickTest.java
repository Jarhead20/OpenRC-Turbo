package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class MotorJoystickTest extends LinearOpMode {
    private DcMotorEx motor;

    @Override
    public void runOpMode(){
        motor = hardwareMap.get(DcMotorEx.class,"Motor");
        waitForStart();

        while (opModeIsActive()) {
            motor.setPower(Math.pow(gamepad1.left_stick_y, 2));
        }
    }

}
