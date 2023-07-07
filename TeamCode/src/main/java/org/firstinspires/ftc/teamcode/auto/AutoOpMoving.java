package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Arm;
import org.firstinspires.ftc.teamcode.Vector2;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

/**
 * A subclass of AutoOp for any AutoOpMode that isn't AutoPark
 */
public abstract class AutoOpMoving extends AutoOp {
    protected Trajectory traj;
    protected Trajectory park1;
    protected Trajectory park2;
    protected Trajectory park3;

    public enum AutoState {
        START,
        PARK1,
        ARMUP,
        PRELOAD,
        CYCLE1,
        CYCLE2,
        CYCLE3,
        CYCLE4,
        PARK2,
        STOP,
    }
    AutoState autoState = AutoState.START;

    int offset;
    int pickHeight;

    // first timer is included in AutoOp
    ElapsedTime timer2 = new ElapsedTime();
    ElapsedTime timer3 = new ElapsedTime();
    Arm arm;
    double downAmount = 30;

    Vector2 pickup1;
    Vector2 pickupGrab;
    Vector2 pickupUp;
    Vector2 depositLoc;

    public AutoOpMoving(int offset, int pickHeight) {
        this.offset = offset;
        this.pickHeight = pickHeight;

        pickup1 = new Vector2(-450+offset, pickHeight);
        pickupGrab = new Vector2(-600+offset, pickHeight);
        pickupUp = new Vector2(-500+offset, 400);
        depositLoc = new Vector2(170, 690);
    }

    protected abstract void setupTrajectories();

    void initComponents() {
        arm = new Arm(hardwareMap, telemetry, timer);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        setupTrajectories();
        arm.closeGripper();

        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(camera, 500);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    private boolean inRange(ElapsedTime timer, double time){
        double low = time-0.1;
        double high = time+0.1;
        return (timer.seconds() >= low && timer.seconds() <= high);
    }
}
