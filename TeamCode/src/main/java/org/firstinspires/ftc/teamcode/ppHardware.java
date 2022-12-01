package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

// Hardware class for the Greenhill 3 testbot
// This hardware is for INFERNO REBORN, the robot we were working on as of January 8, 2022.
public class ppHardware {
    /* Public OpMode members. */
    public DcMotor  frontLeft   = null;
    public DcMotor frontRight  = null;
    public DcMotor backLeft   = null;
    public DcMotor backRight  = null;
    public ColorSensor colorSensor = null;
//    public DcMotor liftRight = null;
//    public DcMotor liftLeft = null;
//    public Servo inRight = null;
//    public Servo inLeft = null;
    public DistanceSensor sensorRange = null;
    //public DcMotor  pulleyMotor0 = null, pulleyMotor1=null, carousel = null;
    //public CRServo extenderServo = null;
//    public Servo clawServo = null, chuteServo = null;
//
//    public TouchSensor magStopBottom = null, magStopMid = null, magStopTop = null;
//
//    public DistanceSensor backDist = null, clawDist = null, strafeDist = null;

//    public static final double grabber_min = 0;
//    public static final double grabber_max = 0.75;

    /* Constructor */
    public ppHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap) {

        // Define and Initialize Devices
        frontLeft  = hwMap.get(DcMotor.class, "frontLeft");
        frontRight = hwMap.get(DcMotor.class, "frontRight");
        backLeft   = hwMap.get(DcMotor.class, "backLeft");
        backRight  = hwMap.get(DcMotor.class, "backRight");

        colorSensor = hwMap.get(ColorSensor.class, "colorSensor");

//       liftLeft = hwMap.get(DcMotor.class,"liftLeft");
//        liftRight = hwMap.get(DcMotor.class,"liftRight");
////
//        inLeft = hwMap.get(Servo.class,"inLeft");
//        inRight = hwMap.get(Servo.class,"inRight");

        sensorRange = hwMap.get(DistanceSensor.class, "sensorRange");


        // Set Direction
        frontLeft.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        frontRight.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        backLeft.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        backRight.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

//        rotateLeft.setDirection(DcMotor.Direction.FORWARD);
//        rotateRight.setDirection(DcMotor.Direction.REVERSE);
//
////        liftMotor.setDirection(DcMotor.Direction.FORWARD);//currently a guess
//        duckSpinner.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set all motors to zero power
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0); // u know who wont be back tho? u.

//
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



    }
}
