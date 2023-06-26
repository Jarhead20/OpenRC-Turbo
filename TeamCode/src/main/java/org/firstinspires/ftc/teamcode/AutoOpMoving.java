package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * A subclass of AutoOp for any AutoOpMode that isn't AutoPark
 */
public class AutoOpMoving extends AutoOp {
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
}
