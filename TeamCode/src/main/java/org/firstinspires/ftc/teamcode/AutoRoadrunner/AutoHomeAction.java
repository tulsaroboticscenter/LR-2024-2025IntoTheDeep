//package org.firstinspires.ftc.teamcode.AutoRoadrunner;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.canvas.Canvas;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.OpenCV.SampleDetectionPipelinePNP;
//import org.openftc.easyopencv.OpenCvWebcam;
//
//import java.util.ArrayList;
//import java.util.concurrent.TimeUnit;
//
//public class AutoHomeAction implements Action {
//    //        public final TimeTrajectory timeTrajectory;
//    private double beginTs = -1;
//    private OpenCvWebcam camera;
//    private SampleDetectionPipelinePNP pipeline;
//    private double startHeading = 0;
//    private LinearOpMode opMode;
//    private MecanumDrive drive;
//    private boolean skipRest = true;
//    private double initY;
//    private ElapsedTime time;
//
////        private final double[] xPoints, yPoints;
//
//    public AutoHomeAction(MecanumDrive drive, OpenCvWebcam camera, SampleDetectionPipelinePNP pipeline, LinearOpMode opMode) {
//        this.drive = drive;
//        this.camera = camera;
//        this.pipeline = pipeline;
//        this.opMode = opMode;
//        this.initY = drive.pose.position.y;
//
//        time = new ElapsedTime();
//        time.reset();
//
//        startHeading = this.drive.pose.heading.toDouble();
//    }
//
//    @Override
//    public boolean run(@NonNull TelemetryPacket p) {
//        ArrayList<SampleDetectionPipelinePNP.AnalyzedStone> samples = pipeline.getDetectedStones();
//
//        if (samples.isEmpty()) {
//            this.drive.leftFront.setPower(0);
//            this.drive.leftBack.setPower(0);
//            this.drive.rightBack.setPower(0);
//            this.drive.rightFront.setPower(0);
//
//            return false;
//        }
//
//        double sampleX = samples.get(0).tvec.get(0, 0)[0] * this.drive.PARAMS.senseInMultX;
//        double sampleY = samples.get(0).tvec.get(1, 0)[0] * this.drive.PARAMS.senseInMultY;
//
//        if(Math.abs(sampleY) >= 10 || Math.abs(sampleX) >= 10) {
//            this.drive.leftFront.setPower(0);
//            this.drive.leftBack.setPower(0);
//            this.drive.rightBack.setPower(0);
//            this.drive.rightFront.setPower(0);
//
//            return false;
//        }
//
//        opMode.telemetry.addData("sampleX: ", sampleX);
//        opMode.telemetry.addData("sampleY: ", sampleY);
//
//        double targetX = this.drive.pose.position.x - (drive.PARAMS.sampleXTargetIn - sampleX);
//        double targetY = this.drive.pose.position.y + (drive.PARAMS.sampleYTargetIn - sampleY);
//
//        Vector2d sampleErrorPose = new Vector2d(drive.PARAMS.sampleXTargetIn - sampleX, initY);
//        double xError = drive.PARAMS.sampleXTargetIn - sampleX;
//        double yError = drive.PARAMS.sampleYTargetIn - sampleY;
//
//        if(Math.abs(xError) < .35 && Math.abs(yError) < .35 || time.time(TimeUnit.SECONDS) >= .5) {
//            this.drive.leftFront.setPower(0);
//            this.drive.leftBack.setPower(0);
//            this.drive.rightBack.setPower(0);
//            this.drive.rightFront.setPower(0);
//            return false;
//        }
//
//        opMode.telemetry.addData("x error: ", xError);
//        opMode.telemetry.addData("y error: ", yError);
//        opMode.telemetry.addData("time: ", time.time(TimeUnit.SECONDS));
//        opMode.telemetry.update();
//
//        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                .strafeToLinearHeading(new Vector2d(targetX, targetY), Math.toRadians(90))
//                .build()
//        );
//
//        return true;
//    }
//
//    @Override
//    public void preview(Canvas c) {
//        c.setStroke("#4CAF507A");
//        c.setStrokeWidth(1);
//    }
//}
