package org.firstinspires.ftc.teamcode.drive.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "RedLeftMiddleApril", group = "BLUE")
public class RedLeftMiddleApril extends LinearOpMode {

DcMotorEx Arm;
Servo Box;
Servo YPD;
Servo intakeLeft;
Servo intakeRight;
CRServo Intake;
    OpenCvCamera webcam;
    GamePropLeft.gamePropPosition propPosition = GamePropLeft.gamePropPosition.LEFT;
    SampleMecanumDrive robot;
    int distancePark;
    int tagOfInterest = 0;
    AprilTagDetection detectedTag = null;
    @Override
    public void runOpMode() throws InterruptedException {

        //arm=new armsNStuff(hardwareMap);

        Arm=hardwareMap.get(DcMotorEx.class, "Arm");
        Box=hardwareMap.get(Servo.class, "Box");
        intakeLeft=hardwareMap.get(Servo.class, "intakeLeft");
        intakeRight=hardwareMap.get(Servo.class, "intakeRight");
        YPD=hardwareMap.get(Servo.class, "YPD");
        Intake=hardwareMap.get(CRServo.class, "Intake");
        Arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setTargetPositionTolerance(10);
        Arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Box.setPosition(0.84);
        YPD.setPosition(0.87);

        telemetry.addData("Start OpMode", "BLUE LEFT");
        telemetry.update();
        startCamera();
        telemetry.addData("Selected Starting Position", propPosition);
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        while (!isStopRequested() && !opModeIsActive())
        {propPosition = getPropPosition();

            telemetry.addData("Identified Prop Location", propPosition);
            telemetry.update();
        }

        if (opModeIsActive() && !isStopRequested()) {
            //Build parking trajectory based on last detected target by vision
            webcam.stopStreaming(); //Stop Webcam to preserve Controlhub cycles.
            runAutonoumousMode();
        }
    }

    public void runAutonoumousMode() {

        Pose2d initPose = new Pose2d(0, 0, Math.toRadians(0.0)); // Starting Pose --Update CoOrdinates
        Pose2d nitPose = new Pose2d(0, 0, Math.toRadians(0.0));
        Pose2d innitPose = new Pose2d(0, 0, Math.toRadians(0.0)); // Starting Pose --Update CoOrdinates
        Pose2d wallPose1 = new Pose2d(0, 0, 0);
        Pose2d aprilTagPose = new Pose2d(0, 0, 0);
        Pose2d wallPose2 = new Pose2d(0, 0, 0);
        Pose2d purplePixelPose = new Pose2d(0, 0, Math.toRadians(0.0));
        Pose2d leftPose  = new Pose2d(0, 0, Math.toRadians(0.0));
        switch (propPosition) { //UPDATE THESE POSITIONS
            case LEFT:
                telemetry.addData("Left", "Left");
                wallPose1 = new Pose2d(46,9, Math.toRadians(-90));
                wallPose2 = new Pose2d(46,17.5, Math.toRadians(-90));
                nitPose = new Pose2d(53,-82, Math.toRadians(-90));
                aprilTagPose = new Pose2d(42.5,-80, Math.toRadians(-90));
                innitPose = new Pose2d(42.5,-90, Math.toRadians(-92.5));
                leftPose = new Pose2d(40,8.5, Math.toRadians(0));
                purplePixelPose = new Pose2d(40.5,8.5, Math.toRadians(0));
                tagOfInterest=4;
                break;
            case CENTER:
                telemetry.addData("Center Position", "Center");
                wallPose1 = new Pose2d(46,9, Math.toRadians(-90));
                wallPose2 = new Pose2d(46,16, Math.toRadians(-90));
                nitPose = new Pose2d(56,-82, Math.toRadians(-90));
                aprilTagPose = new Pose2d(36.5,-80, Math.toRadians(-90));
                innitPose = new Pose2d(36.5,-90, Math.toRadians(-92.5));
                purplePixelPose = new Pose2d(48, -3, Math.toRadians(0.0));
                leftPose = new Pose2d(46.5, -3, Math.toRadians(0.0));
                tagOfInterest=5;
                break;
            case RIGHT:
                wallPose1 = new Pose2d(50,8, Math.toRadians(-80));
                wallPose2 = new Pose2d(50,16, Math.toRadians(-80));
                nitPose = new Pose2d(52,-82, Math.toRadians(-90));
                aprilTagPose = new Pose2d(31.3,-80, Math.toRadians(-90));
                innitPose = new Pose2d(31.3,-90, Math.toRadians(-92.5));
                purplePixelPose = new Pose2d(27, -3, Math.toRadians(90.0));
                leftPose = new Pose2d(27, 2, Math.toRadians(0.0));
                tagOfInterest=6;
                break;

        }
      //  Pose2d parkingPose = new Pose2d(20, 40, -90); //UPDATE

        telemetry.update();
        sleep(200);

        robot = new SampleMecanumDrive(hardwareMap);
        robot.followTrajectorySequence(robot.trajectorySequenceBuilder(initPose) //Starting Pose
                .lineToLinearHeading(leftPose)
                .lineToLinearHeading(purplePixelPose)//Drop Purple Pixel
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {Arm.setTargetPosition(0);Arm.setPower(0.2);Arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);})
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {intakeLeft.setPosition(0.475);})
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {intakeRight.setPosition(0.375);})
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {Intake.setPower(-0.3);})
                .waitSeconds(0.75)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {intakeLeft.setPosition(0);})
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {intakeRight.setPosition(1);})
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {Intake.setPower(0);})
                .forward(8)
                .lineToLinearHeading(wallPose1)
                .lineToLinearHeading(wallPose2)
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {Arm.setTargetPosition(190);Arm.setPower(0.2);Arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);})
                .waitSeconds(.5)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {Intake.setPower(0.75);})
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(36.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/5.0))
                .back(2)
                .resetConstraints()
                .waitSeconds(0.8)
                /*.UNSTABLE_addTemporalMarkerOffset(0.0, () -> {intakeLeft.setPosition(0.62);})
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {intakeRight.setPosition(0.1);})
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {intakeLeft.setPosition(0);})
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {intakeRight.setPosition(1);})*/
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {Intake.setPower(0);})
                //.UNSTABLE_addTemporalMarkerOffset(0.1, () -> {YPD.setPosition(0.3);})
                .resetConstraints()
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(80.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(nitPose)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {Intake.setPower(0.75);})
                .lineToLinearHeading(aprilTagPose)
                .build());
        startApriltagDetection();

        if(detectedTag != null){
            switch (propPosition){
                case LEFT:
                    if (detectedTag.pose.x < 0.6) {
                        innitPose = new Pose2d(41.5,-90, Math.toRadians(-92.5));
                    }
                    if (detectedTag.pose.x > 1)    {
                        innitPose = new Pose2d(43.5,-90, Math.toRadians(-92.5));
                    }
                    break;
                case CENTER:
                    if (detectedTag.pose.x < 0.7) {
                        innitPose = new Pose2d(35.5,-90, Math.toRadians(-92.5));
                    }
                    if (detectedTag.pose.x > 0.85)    {
                        innitPose = new Pose2d(37.5,-90, Math.toRadians(-92.5));
                    }
                    break;
                case RIGHT:
                    if (detectedTag.pose.x < 0.5) {
                        innitPose = new Pose2d(30.3,-90, Math.toRadians(-92.5));
                    }
                    if (detectedTag.pose.x > 0.7)    {
                        innitPose = new Pose2d(32.3,-90, Math.toRadians(-92.5));
                    }
                    break;
            }
        }
        robot.followTrajectorySequence(robot.trajectorySequenceBuilder(aprilTagPose) //Starting Pose
                .lineToLinearHeading(innitPose)
                .resetConstraints()
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/5.0))
                .forward(5)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {YPD.setPosition(0.11);})
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {Arm.setTargetPosition(3200);Arm.setPower(0.7);Arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);})
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {Box.setPosition(0.3);})
                .waitSeconds(3.5)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {Intake.setPower(-0.2);})
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {YPD.setPosition(0.84);})
                .resetConstraints()
                // .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {Intake.setPower(-0.2);})
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {Intake.setPower(0);})
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {Box.setPosition(0.84);})
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {Arm.setTargetPosition(0);Arm.setPower(0.5);Arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);})
                .waitSeconds(3)
                .build());

    }
    public void startCamera() {
        //Initialize Camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "FrontCam"), cameraMonitorViewId);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        webcam.setPipeline(new GamePropLeft());
    }

    public void startApriltagDetection() {
        //Initialize Camera
        double fx = 578.272;
        double fy = 578.272;
        double cx = 402.145;
        double cy = 221.506;

        // UNITS ARE METERS
        double tagsize = 0.166;

        boolean tagFound = false;
        AprilTagDetectionPipeline aprilTagDetectionPipeline;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId1", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "FrontCam"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
        switch (propPosition) { //UPDATE THESE POSITIONS
            case LEFT:
                tagOfInterest = 1;
                break;
            case CENTER:
                tagOfInterest = 2;
                break;
            case RIGHT:
                tagOfInterest = 3;
                break;
        }
        for(AprilTagDetection tag : currentDetections)
        {
            if(tag.id == tagOfInterest)
            {
                tagFound = true;
                detectedTag = tag;
                break;
            }
            telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");

        }

    }

    public GamePropLeft.gamePropPosition getPropPosition() {
        return GamePropLeft.position;
    }

}
