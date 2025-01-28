package org.firstinspires.ftc.teamcode.Misc.P2P;

public class PathGroup {
    private Path[] paths;

    public PathGroup(Path... paths) {
        this.paths = paths;
    }

    public Path[] getPaths() {
        return paths;
    }
}
