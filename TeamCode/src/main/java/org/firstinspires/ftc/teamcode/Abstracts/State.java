package org.firstinspires.ftc.teamcode.Abstracts;

import org.firstinspires.ftc.teamcode.Hardware.Params;

import java.util.HashMap;

public abstract class State {
    public abstract void update();
    public abstract void setParameter(String key, Object value);
    public abstract Object getParameter(String key);
}
