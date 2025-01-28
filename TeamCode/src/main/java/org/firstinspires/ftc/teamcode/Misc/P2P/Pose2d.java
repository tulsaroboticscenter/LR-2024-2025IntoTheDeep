package org.firstinspires.ftc.teamcode.Misc.P2P;

public class Pose2d {
    private double x = 0;
    private double y = 0;
    private double heading = 0;

    public Pose2d(double x, double y, double deg) {
        this.x = x;
        this.y = y;
        this.heading = deg;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() {
        return heading;
    }
}
