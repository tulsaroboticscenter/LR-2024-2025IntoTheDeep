package org.firstinspires.ftc.teamcode.Misc.P2P;

import java.util.HashMap;

public class Path {
    private double xTarget = 0;
    private double yTarget = 0;
    private double headingTarget = 0;
    private HashMap<Double, Runnable> timedRunnables = new HashMap<>();

    public Path(double xTarget, double yTarget, double headingTarget) {
        this.xTarget = xTarget;
        this.yTarget = yTarget;
        this.headingTarget = headingTarget;
    }

    public HashMap<Double, Runnable> getTimedRunnables() {
        return timedRunnables;
    }

    public Path runAfterTime(double time, Runnable runnable) {
        timedRunnables.put(time, runnable);
        return this;
    }

    public double getXTarget() {
        return xTarget;
    }
    public double getYTarget() {
        return yTarget;
    }
    public double getHeadingTarget() {
        return headingTarget;
    }
}
