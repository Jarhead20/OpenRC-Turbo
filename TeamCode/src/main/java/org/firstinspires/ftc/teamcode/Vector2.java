package org.firstinspires.ftc.teamcode;

public class Vector2 {
    public double x;
    double y;
    private final double initX;
    private final double initY;

    public Vector2(double x, double y){
        this.x = x;
        this.y = y;
        this.initX = x;
        this.initY = y;
    }

    public Vector2 lower(double height){
        return new Vector2(this.x, this.y - height);
    }

    public void reset(){
        this.x = initX;
        this.y = initY;
    }

    @Override
    public String toString(){
        return "{x: " + x + " y: " + y + "}";
    }
}
