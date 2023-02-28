//package org.firstinspires.ftc.teamcode.util;
//
//
//import com.arcrobotics.ftclib.command.CommandBase;
//import org.openftc.apriltag.AprilTagDetection;
//
//import java.util.ArrayList;
//
//public class DetectSleeve extends CommandBase {
//
//    public static int zone = -1;
//
//    public DetectSleeve(){
//
//    }
//
//    @Override
//    public void initialize() {
//
//    }
//
//    @Override
//    public boolean isFinished() {
//        //finish after 1 second
//        //the camera automatically stops tracking when goTo for turret is called
//        if(delay != -1){
//            return time.seconds()>=delay;
//        }
//        return time.seconds() >= 1;
//    }
//    ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
//
//            if (currentDetections.size() != 0) {
//        boolean tagFound = false;
//
//        for (AprilTagDetection tag : currentDetections) {
//            if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
//                tagOfInterest = tag;
//                tagFound = true;
//                break;
//            }
//        }
//        if(tagFound)
//        {
//            telemetry.addData("Tag:", tagOfInterest.id);
//            telemetry.addData("initialized", true);
//        }
//    }
//}
