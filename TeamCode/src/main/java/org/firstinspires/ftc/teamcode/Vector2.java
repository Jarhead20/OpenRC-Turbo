package org.firstinspires.ftc.teamcode;

public class Vector2 {
    public double x = 0;
    double y = 0;
    public Vector2(double x, double y){
        this.x = x;
        this.y = y;
    }

    public Vector2 lower(double height){
        return new Vector2(this.x, this.y - height);
    }

    @Override
    public String toString(){
        return "{x: " + x + " y: " + y + "}";
    }
}
