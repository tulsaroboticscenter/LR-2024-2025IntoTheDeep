package org.firstinspires.ftc.teamcode.Hardware.States;

import org.firstinspires.ftc.teamcode.Abstracts.State;
import org.firstinspires.ftc.teamcode.Hardware.Params;

import java.util.HashMap;
import java.util.Map;

public class Intake extends State {
    public double test = 0;
    private Map<String, Object> stateParams = new HashMap<>();
    private Params globalParams;

    public void init() {
        globalParams = new Params();

        stateParams.put("test: ", "test");
        stateParams.put("intakePos: ", 0);
    }

    @Override
    public void update() {

    }

    @Override
    public void setParameter(String key, Object value) {
        stateParams.put(key, value);
    }

    @Override
    public Object getParameter(String key) {
        return stateParams.get(key);
    }
}

