package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="NikitaTest", group="Iterative OpMode")
public class SimpleWristTest extends OpMode {
    // control the 2 servos with joysticks and gripper with a/b
    private Servo gripper;
    private Servo servo1;
    private Servo servo2;

    @Override
    public void init() {
        gripper = hardwareMap.get(Servo.class, "gripper");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        telemetry.addData("Status", "Initialised");
    }

    @Override
    public void loop() {
        // gamepads are -1 to 0
        // servos are 0 to 1

        double roll = gamepad2.right_stick_x * 270;
        double pitch = (gamepad2.right_stick_y + 1) * 270 - 270;
        double servo1Position = (roll + pitch) / 2;
        double servo2Position = (roll - pitch) / 2;
        servo1Position = (servo1Position + 270.0) / 480.0;
        servo2Position = (servo2Position + 270.0) / 480.0;
        telemetry.addData("servo1Position", servo1Position);
        telemetry.addData("servo2Position", servo2Position);
        servo1.setPosition(servo1Position);
        servo2.setPosition(servo2Position);
        //test
        // 0.3 and 0.7
        if(gamepad1.a) {
            gripper.setPosition(0.3);
        } else if (gamepad1.b) {
            gripper.setPosition(0.7);
        }
    }
}
