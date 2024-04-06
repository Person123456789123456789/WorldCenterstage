package org.firstinspires.ftc.teamcode.drive.Auto;

import static java.lang.Math.abs;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;
import org.openftc.easyopencv.OpenCvPipeline;

public class GamePropLeft extends OpenCvPipeline {

    enum gamePropPosition {
        LEFT, CENTER, RIGHT
    }

    TelemetryImpl telemetry;

    //RYAN: Change the co-ordinates for LEFT Point
    Point Mid_pointA = new Point(750, 500);
    Point Mid_pointB = new Point(800,600);

    Point Left_pointA =  new Point(350, 525);
    Point Left_pointB =  new Point(450, 625);

    Point Right_pointA = new Point(875, 325);
    Point Right_pointB = new Point(975,425);


    public static GamePropLeft.gamePropPosition position = GamePropLeft.gamePropPosition.LEFT; //Default Position

    Mat mid_Hue = null;
    Mat left_Hue = null;
    Mat right_Hue = null;
    Mat HSV = new Mat();
    Mat hue = new Mat();
    int avg1, avg2, avg3;
    Scalar RED = new Scalar(255.0, 0.0, 0.0);
    Scalar BLUE = new Scalar(0.0, 0.0, 255.0);

    public void inputToHue(Mat input) {
        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);
        Core.extractChannel(HSV, hue, 1); //Gets the HUE value out
    }

    @Override
    public void init(Mat firstFrame) {
        inputToHue(firstFrame);


        left_Hue = hue.submat(new Rect(Left_pointA, Left_pointB));
        mid_Hue = hue.submat(new Rect(Mid_pointA, Mid_pointB));
        right_Hue = hue.submat(new Rect(Right_pointA, Right_pointB));
    }

    @Override
    public Mat processFrame(Mat input) {
        inputToHue(input);

        avg1 = (int) Core.mean(left_Hue).val[0];
        avg2 = (int) Core.mean(mid_Hue).val[0];
        avg3 = abs(avg1 - avg2);

        int max = Math.max(avg1, avg2);

        Imgproc.rectangle(
                input,
                Left_pointA,
                Left_pointB,
                RED, 4);
        Imgproc.rectangle(
                input,
                Mid_pointA,
                Mid_pointB,
                RED, 4);


        if (avg3 < 50) {
            position = GamePropLeft.gamePropPosition.RIGHT;
        } else if (max == avg2) {
            position = GamePropLeft.gamePropPosition.CENTER;
        } else {
            position = GamePropLeft.gamePropPosition.LEFT;
        }
        //telemetry.addData("[avg1]", avg1);
        // telemetry.addData("[avg2]", avg2);
        //telemetry.addData("[avg3]", avg3);
        //telemetry.addData("Position", position);

        // telemetry.update();
        return input;
    }
}