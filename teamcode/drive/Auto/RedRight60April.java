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

@Autonomous(name = "RedRight60April", group = "BLUE")
public class RedRight60April extends LinearOpMode {

DcMotorEx Arm;
Servo Box;
Servo YPD;
Servo intakeLeft;
Servo intakeRight;
CRServo Intake;
    OpenCvCamera webcam;
    GamePropRight.gamePropPosition propPosition = GamePropRight.gamePropPosition.LEFT;
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
        YPD.setPosition(0.84);

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
        Pose2d yellowPixelPose = new Pose2d(0, 0, 0);
        Pose2d returnPose = new Pose2d(0, 0, 0);
        Pose2d aprilTagPose = new Pose2d(0, 0, 0);
        Pose2d wallPose = new Pose2d(0, 0, 0);
        Pose2d wallPose2 = new Pose2d(0, 0, 0);
        Pose2d place = new Pose2d(0, 0, 0);
        Pose2d leftPose = new Pose2d(0, 0, 0);
        Pose2d purplePixelPose = new Pose2d(0, 0, Math.toRadians(0.0));;
        switch (propPosition) { //UPDATE THESE POSITIONS
            case LEFT:
                telemetry.addData("Left Position", "Left");
                yellowPixelPose = new Pose2d(37,-34, Math.toRadians(-90));
                aprilTagPose = new Pose2d(37,-28, Math.toRadians(-90));
                purplePixelPose = new Pose2d(26, 2, Math.toRadians(-90.0));
                returnPose = new Pose2d(47, -7, Math.toRadians(-90));
                wallPose = new Pose2d(47, 30, Math.toRadians(-85));
                wallPose2 = new Pose2d(40, 72, Math.toRadians(-85));
                place = new Pose2d(30, -34, Math.toRadians(-90));
                tagOfInterest=4;
                break;
            case CENTER:
                telemetry.addData("Center Position", "Center");
                yellowPixelPose = new Pose2d(30,-34.5 ,Math.toRadians(-90));
                aprilTagPose = new Pose2d(30,-28, Math.toRadians(-90));
                purplePixelPose = new Pose2d(33, -10, Math.toRadians(-90.0));
                returnPose = new Pose2d(47, -7, Math.toRadians(-90));
                wallPose = new Pose2d(47, 30, Math.toRadians(-85));
                wallPose2 = new Pose2d(40, 72, Math.toRadians(-85));
                place = new Pose2d(30, -34, Math.toRadians(-90));
                tagOfInterest=5;
                break;
            case RIGHT:
                telemetry.addData("Right Position", "Right");
                yellowPixelPose = new Pose2d(23.5, -34, Math.toRadians(-90));
                aprilTagPose = new Pose2d(23.5,-28, Math.toRadians(-90));
                returnPose = new Pose2d(47, -25, Math.toRadians(-90));
                wallPose = new Pose2d(47, 30, Math.toRadians(-85));
                wallPose2 = new Pose2d(40, 72, Math.toRadians(-85));
                place = new Pose2d(30, -34, Math.toRadians(-90));
                purplePixelPose = new Pose2d(30,-17.5, Math.toRadians(-90));
                tagOfInterest=6;
                break;

        }
      //  Pose2d parkingPose = new Pose2d(20, 40, -90); //UPDATE

        telemetry.update();
        sleep(200);

        robot = new SampleMecanumDrive(hardwareMap);
        robot.followTrajectorySequence(robot.trajectorySequenceBuilder(initPose) //Starting Pose
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {Arm.setTargetPosition(300);Arm.setPower(0.8);Arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);})
                .lineToLinearHeading(aprilTagPose)
                .build());
        startApriltagDetection();

        if(detectedTag != null){
            switch (propPosition){
                case LEFT:
                    if (detectedTag.pose.x < 0.6) {
                        yellowPixelPose = new Pose2d(36,-34, Math.toRadians(-90));
                    }
                    if (detectedTag.pose.x > 1)    {
                        yellowPixelPose = new Pose2d(38,-34, Math.toRadians(-90));
                    }
                    break;
                case CENTER:
                    if (detectedTag.pose.x < 0.7) {
                        yellowPixelPose = new Pose2d(29,-34.5 ,Math.toRadians(-90));
                    }
                    if (detectedTag.pose.x > 0.85)    {
                        yellowPixelPose = new Pose2d(31,-34.5 ,Math.toRadians(-90));
                    }
                    break;
                case RIGHT:
                    if (detectedTag.pose.x < 0.5) {
                        yellowPixelPose = new Pose2d(22.5, -34, Math.toRadians(-90));
                    }
                    if (detectedTag.pose.x > 0.7)    {
                        yellowPixelPose = new Pose2d(24.5, -34, Math.toRadians(-90));
                    }
                    break;
            }
        }
        robot.followTrajectorySequence(robot.trajectorySequenceBuilder(aprilTagPose) //Starting Pose
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(50.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {Arm.setTargetPosition(170);Arm.setPower(0.8);Arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);})
                .lineToLinearHeading(yellowPixelPose)
                .forward(3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {YPD.setPosition(0.11);})
                .waitSeconds(1.2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {YPD.setPosition(0.84);})
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {Arm.setTargetPosition(150);Arm.setPower(0.4);Arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);})
                .lineToLinearHeading(purplePixelPose)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {Arm.setTargetPosition(0);Arm.setPower(0.4);Arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);})
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {intakeLeft.setPosition(0.475);})
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {intakeRight.setPosition(0.375);})
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {Intake.setPower(-0.3);})
                .waitSeconds(0.75)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {intakeLeft.setPosition(0);})
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {intakeRight.setPosition(1);})
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {Intake.setPower(0);})
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {Arm.setTargetPosition(0);Arm.setPower(0.2);Arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);})
                .lineToLinearHeading(returnPose)
                .lineToLinearHeading(wallPose)
                .lineToLinearHeading(wallPose2)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {Arm.setTargetPosition(130);Arm.setPower(0.2);Arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);})
                .waitSeconds(.35)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {Intake.setPower(0.75);})
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(40.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/5.0))
                .back(4)
                .waitSeconds(0.75)
                .resetConstraints()
                .lineToLinearHeading(wallPose)
                .lineToLinearHeading(returnPose)
                .lineToLinearHeading(place)
                //.UNSTABLE_addTemporalMarkerOffset(0.1, () -> {Arm.setTargetPosition(0);Arm.setPower(0.65);Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);})
                .resetConstraints()
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2.0))
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {Arm.setTargetPosition(3100);Arm.setPower(1);Arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);})
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {Box.setPosition(0.34);})
                .back(3)
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {Intake.setPower(-0.2);})
                .resetConstraints()
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {Intake.setPower(0);})
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {Box.setPosition(0.84);})
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {Arm.setTargetPosition(0);Arm.setPower(1);Arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);})
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

        webcam.setPipeline(new GamePropRight());
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

    public GamePropRight.gamePropPosition getPropPosition() {
        return GamePropRight.position;
    }

}
