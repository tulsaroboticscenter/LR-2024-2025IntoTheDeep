package org.firstinspires.ftc.teamcode.Abstracts;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class AutoProgram {
    public abstract void init(LinearOpMode _opMode);
    public abstract void startAuto();
    public abstract String getAutoName();
}
