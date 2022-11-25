package org.firstinspires.ftc.teamcode;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.ppHardware;
import org.firstinspires.ftc.teamcode.ppDriving;


@Autonomous(name=" color auton neha")



//NEED TO CORRECT THE MOVEMENT AND ORIENTATION BEFORE RUNNING. IT YEETS ITSELF FORWARD WHEN RUN
    public class colorAuton extends ppDriving {
    public int x;
    public int y;

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    final double DESIRED_DISTANCE = 8.0; //  this is how close the camera should get to the target (inches)
    //  The GAIN constants set the relationship between the measured position error,
    //  and how much power is applied to the drive motors.  Drive = Error * Gain
    //  Make these values smaller for smoother control.
    final double SPEED_GAIN = 0.02;   //  Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MM_PER_INCH = 25.40;   //  Metric conversion
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    public DcMotor liftLeft = null;
    public DcMotor liftRight = null;
    private ColorSensor colorSensor = null;


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        liftLeft = hardwareMap.get(DcMotor.class, "liftLeft");
        liftRight = hardwareMap.get(DcMotor.class, "liftRight");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        // bPrevState and bCurrState represent the previous and current state of the button.
        boolean bPrevState = false;
        boolean bCurrState = false;

        // bLedOn represents the state of the LED.
        boolean bLedOn = true;

        // get a reference to our ColorSensor object.
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");

        // Set the LED in the beginning
        colorSensor.enableLed(bLedOn);

        waitForStart();
        while (opModeIsActive()) {

            // check the status of the x button on either gamepad.
            bCurrState = gamepad1.x;

            // check for button state transitions.
            if (bCurrState && (bCurrState != bPrevState)) {

                // button is transitioning to a pressed state. So Toggle LED
                bLedOn = !bLedOn;
                colorSensor.enableLed(bLedOn);
            }

            // update previous state variable.
            bPrevState = bCurrState;

            // convert the RGB values to HSV values.
            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

            // send the info back to driver station using telemetry function.
            telemetry.addData("LED", bLedOn ? "On" : "Off");
            telemetry.addData("Clear", colorSensor.alpha());
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue ", colorSensor.blue());
            telemetry.addData("Hue", hsvValues[0]);


            boolean colorFound = false;
            while (colorFound == false) {
                //rotate(.6,'r',35);//these rotates are for correctional purposes
                move(.6, 'f', 5);
                    if (Math.abs(colorSensor.green() + (colorSensor.red()/2) - colorSensor.blue()) < 35) {
                        telemetry.addData("PURPLE FOUND",colorSensor.alpha());
                        colorFound = true;
                        move(.6, 'f', 10);

                    }
                    else if (colorSensor.green() > colorSensor.blue() && colorSensor.blue() > colorSensor.red()){
                        telemetry.addData("BLACK FOUND",colorSensor.alpha());
                        colorFound = true;
                        move(.6, 'f', 7);
                        move(.6, 'l',15);
                    }
                    else if (Math.abs(colorSensor.blue() + (colorSensor.red()/2) - colorSensor.green()) < 30) {
                        telemetry.addData("ORANGE FOUND",colorSensor.alpha());
                        colorFound = true;
                        move(.6, 'f', 7);
                        move(.6, 'r',15);
                    }
            }

        }
    }
}

