package org.firstinspires.ftc.teamcode.Abstracts;

import org.firstinspires.ftc.teamcode.Enums.TeleopMode;

public abstract class NewSubsystem {
    public abstract void update();
    public abstract void setState(State mode);

    public abstract State getState();
}
