package org.firstinspires.ftc.teamcode.drive.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.armsNStuff;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
@Autonomous(name = "BlueLeft", group = "BLUE")
public class BLue_Left extends LinearOpMode {

DcMotorEx Arm;
Servo Box;
Servo YPD;
CRServo Intake;
    OpenCvCamera webcam;
    GamePropLeft.gamePropPosition propPosition = GamePropLeft.gamePropPosition.LEFT;
    SampleMecanumDrive robot;
    int distancePark;
    @Override
    public void runOpMode() throws InterruptedException {

        //arm=new armsNStuff(hardwareMap);

        Arm=hardwareMap.get(DcMotorEx.class, "Arm");
        Box=hardwareMap.get(Servo.class, "Box");
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
        Pose2d rightPixelPose = new Pose2d(0, 0, 0);
        Pose2d purplePixelPose = new Pose2d(0, 0, Math.toRadians(0.0));;
        switch (propPosition) { //UPDATE THESE POSITIONS
            case LEFT:
                rightPixelPose = new Pose2d(18.5,7.5, Math.toRadians(0));
                yellowPixelPose = new Pose2d(13.5, 28.5, Math.toRadians(90));
                telemetry.addData("Left Position", "Left");
                purplePixelPose = new Pose2d(21,7.5, Math.toRadians(0));
                distancePark = 15;
                break;
            case CENTER:
                telemetry.addData("Center Position", "Center");
                yellowPixelPose = new Pose2d(21.5,29.5 ,Math.toRadians(90));
                purplePixelPose = new Pose2d(27, -2, Math.toRadians(0.0));
                rightPixelPose = new Pose2d(24.5, -2, Math.toRadians(0.0));
                distancePark = 19;
                break;
            case RIGHT:
                telemetry.addData("Right", "Right");
                yellowPixelPose = new Pose2d(23,29.5, Math.toRadians(90));
                purplePixelPose = new Pose2d(29, -5, Math.toRadians(-90.0));
                rightPixelPose = new Pose2d(29, 1, Math.toRadians(-90.0));
                distancePark = 29;
                break;

        }
      //  Pose2d parkingPose = new Pose2d(20, 40, -90); //UPDATE

        telemetry.update();
        sleep(200);

        robot = new SampleMecanumDrive(hardwareMap);
        robot.followTrajectorySequence(robot.trajectorySequenceBuilder(initPose) //Starting Pose
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(50.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {Arm.setTargetPosition(500);Arm.setPower(0.8);Arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);})
                .lineToLinearHeading(rightPixelPose)
                .lineToLinearHeading(purplePixelPose)//Drop Purple Pixel
                //.UNSTABLE_addTemporalMarkerOffset(0, () -> {2350})
                //.UNSTABLE_addTemporalMarkerOffset(0, () -> {0.45;})
                .back(10)
                .lineToLinearHeading(yellowPixelPose)
                //.waitSeconds(1)
               // .forward(4)
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(17.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/8.0))
                .forward(9.7)
                //.UNSTABLE_addTemporalMarkerOffset(0.0, () -> {Intake.setPower(-0.2);})
                .waitSeconds(.2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {YPD.setPosition(0.11);})
                .waitSeconds(2)
                //.UNSTABLE_addTemporalMarkerOffset(0.1, () -> {Arm.setTargetPosition(0);Arm.setPower(0.65);Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);})
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {YPD.setPosition(0.84);})
                .resetConstraints()
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2.0))
                .back(5)
                .strafeLeft(distancePark)
             //   .lineToLinearHeading(parkingPose) //Parking in the back
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
	
	public GamePropLeft.gamePropPosition getPropPosition() {
		return GamePropLeft.position; 
	}
	
}