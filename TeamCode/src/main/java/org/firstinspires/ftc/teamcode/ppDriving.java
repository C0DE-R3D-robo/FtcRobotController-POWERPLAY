package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public abstract class ppDriving extends LinearOpMode{


    ppHardware robot;   // Use a Pushbot's hardware

    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    static final double     INCHES_FOR_RIGHT_ANGLE  = 4;

    static final double     LIFT_COUNTS_FULL_REVOLVE= 1440 /4;

    final double DESIRED_DISTANCE = 8.0; //  this is how close the camera should get to the target (inches)
    //  The GAIN constants set the relationship between the measured position error,
    //  and how much power is applied to the drive motors.  Drive = Error * Gain
    //  Make these values smaller for smoother control.
    final double SPEED_GAIN =   0.02 ;   //  Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double TURN_GAIN  =   0.01 ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MM_PER_INCH = 25.40 ;   //  Metric conversion
//    private DcMotor backLeft = null;
//    private DcMotor backRight = null;
//    private DcMotor frontLeft = null;
//    private DcMotor frontRight = null;

    public void setRobot(ppHardware robot){
        this.robot = robot;
    }
    public void motorStop() {
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
//        robot.carousel.setPower(0);
        reset();
    }
    public void reset(){
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
//
//    public void distanceMove(double distance, boolean relativeMove){
//        double reversePercent = 1; //speed percentage when going backward
//        double power = .1; //this is the power for the 2022 neverest20 motors
//
//        //measure and calculate the starting and goal distances based on relative and absolute movement
//        //RELATIVE (relative move = true): move 20 cm from where I am now (if you start 10 cm from the wall, you will move 20 cm to a total of 30 cm)
//        //ABSOLUTE (relative move = false): move until you are 20 cm from wall (if you start 10 cm from the wall, you will move 10cm, to a total of 20 cm)
//
//        double startingDist = robot.backDist.getDistance(DistanceUnit.CM);
//        double goalDist = distance;
//        if(relativeMove) {
//            goalDist+=startingDist;
//        }
//
//        long currentTime = System.currentTimeMillis();
//        long timeOutTime = 7000L;
//        long totalTime = currentTime + timeOutTime;
//
//        double currentDist = robot.backDist.getDistance(DistanceUnit.CM);
//
//        while (currentDist<=goalDist){//until the current dist is greater than the goal
//            robot.frontLeft.setPower(power);
//            robot.frontRight.setPower(power);
//            robot.backLeft.setPower(power);
//            robot.backRight.setPower(power);
//            currentDist = robot.backDist.getDistance(DistanceUnit.CM);
//            telemetry.addData("Current Dist", currentDist);
//            telemetry.update();
//
////            if (totalTime < System.currentTimeMillis()){
////                return;
////            }
//        }
//        motorStop();
//        sleep(200);
//        currentDist = robot.backDist.getDistance(DistanceUnit.CM);
//
//        if (currentDist> goalDist+2.0){//this may or may not be working, but the code is precise enough that right now, we don't have to worry XOXO Quyen Feb7, 2022
//            robot.frontLeft.setPower(-power *reversePercent);
//            robot.frontRight.setPower(-power *reversePercent );
//            robot.backLeft.setPower(-power *reversePercent);
//            robot.backRight.setPower(-power *reversePercent);
//            currentDist = robot.backDist.getDistance(DistanceUnit.CM);
//            telemetry.addData("Current Dist (reverse)", currentDist);
//            telemetry.update();
////            if (totalTime < System.currentTimeMillis()){
////                return;
////            }
//        }
//        motorStop();
//    }
//    public void strafeMove(double distance, boolean relativeMove){
//        double reversePercent = 1; //speed percentage when going backward
//        double power = .1; //this is the power for the 2022 neverest20 motors
//
//        //measure and calculate the starting and goal distances based on relative and absolute movement
//        //RELATIVE (relative move = true): move 20 cm from where I am now (if you start 10 cm from the wall, you will move 20 cm to a total of 30 cm)
//        //ABSOLUTE (relative move = false): move until you are 20 cm from wall (if you start 10 cm from the wall, you will move 10cm, to a total of 20 cm)
//
//        double startingDist = robot.backDist.getDistance(DistanceUnit.CM);
//        double goalDist = distance;
//        if(relativeMove) {
//            goalDist+=startingDist;
//        }
//
//        long currentTime = System.currentTimeMillis();
//        long timeOutTime = 7000L;
//        long totalTime = currentTime + timeOutTime;
//
//        double currentDist = robot.strafeDist.getDistance(DistanceUnit.CM);
//
//        while (currentDist<=goalDist){//until the current dist is greater than the goal
//            robot.frontLeft.setPower(power);
//            robot.frontRight.setPower(-power);
//            robot.backLeft.setPower(-power);
//            robot.backRight.setPower(power);
//            currentDist = robot.strafeDist.getDistance(DistanceUnit.CM);
//            telemetry.addData("Current Dist", currentDist);
//            telemetry.update();
//
////            if (totalTime < System.currentTimeMillis()){
////                return;
////            }
//        }
//        motorStop();
//        sleep(200);
//        currentDist = robot.strafeDist.getDistance(DistanceUnit.CM);
//
//        if (currentDist> goalDist+2.0){//this may or may not be working, but the code is precise enough that right now, we don't have to worry XOXO Quyen Feb7, 2022
//            robot.frontLeft.setPower(-power *reversePercent);
//            robot.frontRight.setPower(power *reversePercent );
//            robot.backLeft.setPower(power *reversePercent);
//            robot.backRight.setPower(-power *reversePercent);
//            currentDist = robot.strafeDist.getDistance(DistanceUnit.CM);
//            telemetry.addData("Current Dist (reverse)", currentDist);
//            telemetry.update();
////            if (totalTime < System.currentTimeMillis()){
////                return;
////            }
//        }
//        motorStop();
//    }
    public void move(double power, char direction, double distance){
        double ticks = COUNTS_PER_INCH * distance;
//        double ticks = 7.5* distance;
        //defining the power variable for later in the cases (these 'max' calculations make sure we drive straight so NO MORE SPINNING!!!)
        double max;
        double flPower  = power;
        double frPower = power;
        double blPower   = power;
        double brPower  = power;
        max = Math.max(Math.abs(flPower), Math.abs(frPower));
        max = Math.max(max, Math.abs(blPower));
        max = Math.max(max, Math.abs(brPower));
        flPower  /= max;
        frPower /= max;
        blPower   /= max;
        brPower  /= max;
        switch(direction){
            case 'f':
                //to go forward

                //set target position
                robot.frontLeft.setTargetPosition((int)ticks);
                robot.backLeft.setTargetPosition((int)ticks);
                robot.frontRight.setTargetPosition((int)ticks);
                robot.backRight.setTargetPosition((int)ticks);
                //set run to position
                robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //set drive power for forward
                robot.frontLeft.setPower(flPower);
                robot.frontRight.setPower(frPower);
                robot.backLeft.setPower(blPower);
                robot.backRight.setPower(brPower);

                while (robot.frontLeft.isBusy() && robot.backLeft.isBusy() && robot.frontRight.isBusy() && robot.backRight.isBusy())
                {
                    telemetry.clear();
                    telemetry.addData("Front Left Pos", robot.frontLeft.getCurrentPosition());
                    telemetry.addData("Front Right Pos", robot.frontRight.getCurrentPosition());
                    telemetry.addData("Back Left Pos", robot.backLeft.getCurrentPosition());
                    telemetry.addData("Back Right (Mephistopheles) Pos", robot.backRight.getCurrentPosition());
                    telemetry.update();
                }
                motorStop();
                robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                break;

            case 'b':
                //setting power of motors to go backward


                //set target position
                robot.frontLeft.setTargetPosition((int)-ticks);
                robot.backLeft.setTargetPosition((int) -ticks);
                robot.frontRight.setTargetPosition((int) -ticks);
                robot.backRight.setTargetPosition((int) -ticks);
                //set run to position
                robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //set drive power for backward
                robot.frontLeft.setPower(-flPower);
                robot.frontRight.setPower(-frPower);
                robot.backLeft.setPower(-blPower);
                robot.backRight.setPower(-brPower);

                while (robot.frontLeft.isBusy() && robot.backLeft.isBusy() && robot.frontRight.isBusy() && robot.backRight.isBusy())
                {
                    telemetry.clear();
                    telemetry.addData("Front Left Pos", robot.frontLeft.getCurrentPosition());
                    telemetry.addData("Front Right Pos", robot.frontRight.getCurrentPosition());
                    telemetry.addData("Back Left Pos", robot.backLeft.getCurrentPosition());
                    telemetry.addData("Back Right (Mephistopheles) Pos", robot.backRight.getCurrentPosition());
                    telemetry.update();

                }
                motorStop();
                robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                break;

            case 'r':
                //to strafe right


                //set target position
                robot.frontLeft.setTargetPosition((int) ticks);
                robot.backLeft.setTargetPosition((int)-ticks);
                robot.frontRight.setTargetPosition((int)-ticks);
                robot.backRight.setTargetPosition((int) ticks);
                //set run to position
                robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //set drive power for forward
                robot.frontLeft.setPower(flPower);
                robot.frontRight.setPower(-frPower);
                robot.backLeft.setPower(-blPower);
                robot.backRight.setPower(brPower);

                while (robot.frontLeft.isBusy() && robot.backLeft.isBusy() && robot.frontRight.isBusy() && robot.backRight.isBusy())
                {

                }
                motorStop();
                robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                break;
            case 'l' :
                // to strafe left

                //set target position
                robot.frontLeft.setTargetPosition((int)-ticks);
                robot.backLeft.setTargetPosition((int)ticks);
                robot.frontRight.setTargetPosition((int)ticks);
                robot.backRight.setTargetPosition((int)-ticks);
                //set run to position
                robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //set drive power for forward
                robot.frontLeft.setPower(-flPower);
                robot.frontRight.setPower(frPower);
                robot.backLeft.setPower(blPower);
                robot.backRight.setPower(-brPower);

                while (robot.frontLeft.isBusy() && robot.backLeft.isBusy() && robot.frontRight.isBusy() && robot.backRight.isBusy())
                {

                }
                motorStop();
                robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                break;

            default:
                motorStop();
        }
    }

    public void rotate(double power, char direction, double angle) {
        double ticks = COUNTS_PER_INCH * angle / 90 * INCHES_FOR_RIGHT_ANGLE;
//        double ticks = 7.5* distance;
        double max;
        switch(direction){
            case 'r':
                //to turn clockwise

                robot.frontLeft.setTargetPosition((int)ticks);
                robot.backLeft.setTargetPosition((int)ticks);
                robot.frontRight.setTargetPosition((int)-ticks);
                robot.backRight.setTargetPosition((int)-ticks);
                //set run to position
                robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                //set drive power for forward
                robot.frontLeft.setPower(power);
                robot.frontRight.setPower(-power);
                robot.backLeft.setPower(power);
                robot.backRight.setPower(-power);

                while (robot.frontLeft.isBusy() && robot.backLeft.isBusy() && robot.frontRight.isBusy() && robot.backRight.isBusy())
                {

                }
                motorStop();
                robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                break;
            case 'l':
                // to turn counter clockwise

                robot.frontLeft.setTargetPosition((int)-ticks);
                robot.backLeft.setTargetPosition((int) -ticks);
                robot.frontRight.setTargetPosition((int)ticks);
                robot.backRight.setTargetPosition((int) ticks);
                //set run to position
                robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //set drive power for forward
                robot.frontLeft.setPower(-power);
                robot.frontRight.setPower(-power);
                robot.backLeft.setPower(power);
                robot.backRight.setPower(power);

                while (robot.frontLeft.isBusy() && robot.backLeft.isBusy() && robot.frontRight.isBusy() && robot.backRight.isBusy())
                {
                    telemetry.clear();
                    telemetry.addData("Front Left Pos", robot.frontLeft.getCurrentPosition());
                    telemetry.addData("Front Right Pos", robot.frontRight.getCurrentPosition());
                    telemetry.addData("Back Left Pos", robot.backLeft.getCurrentPosition());
                    telemetry.addData("Back Right (Mephistopheles) Pos", robot.backRight.getCurrentPosition());
                    telemetry.update();
                }
                motorStop();
                robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                break;
            default:
                motorStop();
        }
    }
    public void diagonal(double power, char direction, long distance){
        double ticks = 1120/7.5 * distance;
        switch(direction) {
            case '1':
                //forward right

                //set target position

                robot.frontLeft.setTargetPosition((int) (ticks));
                robot.backLeft.setTargetPosition(0);
                robot.frontRight.setTargetPosition(0);
                robot.backRight.setTargetPosition((int) ticks);
                //set run to position
                robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //set drive power for forward
                robot.frontLeft.setPower(power);
                robot.frontRight.setPower(0);
                robot.backLeft.setPower(0);
                robot.backRight.setPower(power);

                while (robot.frontLeft.isBusy() && robot.backLeft.isBusy() && robot.frontRight.isBusy() && robot.backRight.isBusy()) {

                }
                motorStop();
                robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                break;

            case '2':
                //forward left

                //set target position

                robot.frontLeft.setTargetPosition(0);
                robot.backLeft.setTargetPosition((int) ticks);
                robot.frontRight.setTargetPosition((int) ticks);
                robot.backRight.setTargetPosition(0);
                //set run to position
                robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //set drive power for forward
                robot.frontLeft.setPower(0);
                robot.frontRight.setPower(power);
                robot.backLeft.setPower(power);
                robot.backRight.setPower(0);

                while (robot.frontLeft.isBusy() && robot.backLeft.isBusy() && robot.frontRight.isBusy() && robot.backRight.isBusy()) {

                }
                motorStop();
                robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


                break;
            case '3':
                // go back right

                robot.frontLeft.setTargetPosition(0);
                robot.backLeft.setTargetPosition((int) -ticks);
                robot.frontRight.setTargetPosition((int) -ticks);
                robot.backRight.setTargetPosition(0);
                //set run to position
                robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //set drive power for forward
                robot.frontLeft.setPower(0);
                robot.frontRight.setPower(-power);
                robot.backLeft.setPower(-power);
                robot.backRight.setPower(0);

                while (robot.frontLeft.isBusy() && robot.backLeft.isBusy() && robot.frontRight.isBusy() && robot.backRight.isBusy()) {

                }
                motorStop();
                robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


                break;

            case '4':
//back left
                robot.frontLeft.setTargetPosition((int) -ticks);
                robot.backLeft.setTargetPosition(0);
                robot.frontRight.setTargetPosition(0);
                robot.backRight.setTargetPosition((int) -ticks);
                //set run to position
                robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //set drive power for forward
                robot.frontLeft.setPower(-power);
                robot.frontRight.setPower(0);
                robot.backLeft.setPower(0);
                robot.backRight.setPower(-power);

                while (robot.frontLeft.isBusy() && robot.backLeft.isBusy() && robot.frontRight.isBusy() && robot.backRight.isBusy()) {

                }
                motorStop();
                robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


                break;
            default:
                motorStop();

        }
    }//hi
    public void armheight(double power) {
        //double ticks = 1120 / 7.5 * distance;

        robot.Claw.setPosition(.10);//close claw on cone
        robot.elbow.setTargetPosition(1100);
        //set run to position
        robot.elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //set drive power for forward
        robot.elbow.setPower(power);//lift up with arm still extended
        while (robot.elbow.isBusy()) {
            telemetry.clear();
            telemetry.addData("LIFT","CURRENTLY LIFTING THE ARM");
            telemetry.update();
        }
//        robot.elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        robot.elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.neck.setTargetPosition(-1000);
//        //set run to position
//        robot.neck.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.clawRotate.setPosition(0);//flip claw
        //set drive power for forward
        while (robot.neck.isBusy() || robot.elbow.isBusy()) {

        }
//        robot.neck.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.Claw.setPosition(.75);//open/drop cone
        sleep(3000);
        robot.clawRotate.setPosition(1);//flip claw back
        robot.Claw.setPosition(.1);//close
        // delay

    }
//    public void lift (double power, int level, long moveTime){
//        robot.rotateLeft.setPower(.05*power);
//        robot.rotateRight.setPower(.05*power);
//
//        long currentTime = System.currentTimeMillis();
//        boolean atPosition = false;
//        while (currentTime + moveTime > System.currentTimeMillis() && !atPosition){
//            telemetry.addData("Target Time", currentTime+moveTime);
//            telemetry.addData("Current time", System.currentTimeMillis());
//            telemetry.update();
//            if (robot.magStopBottom.getValue()==1.0 && level !=1){
//                robot.rotateRight.setPower(-.1);
//                robot.rotateLeft.setPower(-.1);
//                break;
//            }
//            switch (level){
//                case 1:
//                    if (robot.magStopBottom.getValue()==1.0){
//                        robot.rotateRight.setPower(-.37);
//                        robot.rotateLeft.setPower(-.37);
//                        telemetry.addData("Stopped", 1);
//                        telemetry.update();
//                        sleep(200);
//                        motorStop();
//                        robot.rotateRight.setPower(-.05);
//                        robot.rotateRight.setPower(-.05);
//                        sleep(300);
//                        motorStop();
//                        break;
//                    }
//                    break;
//                case 2:
//                    if (robot.magStopMid.getValue()==1.0){
//                        robot.rotateRight.setPower(-.37);
//                        robot.rotateLeft.setPower(-.37);
//                        telemetry.addData("Stopped", 2);
//                        telemetry.update();
//                        sleep(200);
//                        atPosition = true;
//                        break;
//                    }
//                    break;
//                case 3:
//                    if (robot.magStopTop.getValue()==1.0){
//                        sleep(150);
//                        robot.rotateRight.setPower(-.1);
//                        robot.rotateLeft.setPower(-.1);
//                        sleep(50);
//                        atPosition = true;
//                        break;
//                    }
//                    break;
//                default:
//                    break;
//            }
//        }
//
//        motorStop();
//
//
//        //Theoretical way to get it to run on encoders. doesn't work for now.
////        double ticks = LIFT_COUNTS_FULL_REVOLVE * anglePercent /20;
////
////        robot.rotateLeft.setTargetPosition((int)ticks);
////        robot.rotateRight.setTargetPosition((int)ticks);
////
////        robot.rotateLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        robot.rotateRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////
////        robot.rotateLeft.setPower(.02);
////        robot.rotateRight.setPower(.02);
////
////
////        while (robot.rotateLeft.isBusy()){
////            telemetry.addData("Target:", (int)ticks);
////            telemetry.addData("Current", robot.rotateLeft.getCurrentPosition());
////            telemetry.update();
////
////        }
////        telemetry.addData("OUT", "YUP");
////        telemetry.update();
////        motorStop();
//    }
//
//    public void levelLift (char clawTarget){
//        double lowPosition = 90;//target positions, in mm
//        double midPosition = 108;
//        double highPosition = 243;
//
//        double precisePower = .03;//powers of the motor in different modes
//        double roughPower = .05;
//        double correctionSpeed = .1;
//
//        int waitTime = 100;
//        int distanceCutoff = 2000;
//
//        double targetPosition;
//
//        switch (clawTarget){//sets the target based on the user's input
//            case 'l':
//            case 'b':
//                targetPosition = lowPosition;
//                break;
//            case 'm':
//                targetPosition = midPosition;
//                break;
//            case 't':
//            case 'h':
//                targetPosition = highPosition;
//                break;
//            default:
//                targetPosition = 3000;//this is a test number. Should read very high if not pointing at anything.
//        }
//
//        double currentPosition = robot.clawDist.getDistance(DistanceUnit.MM);
//
//        motorStop();
//
//        while (currentPosition - 500 > targetPosition){//if the current position is 50 cm more than the target position, it moves very speedily to get there
//            robot.rotateLeft.setPower(roughPower);//it should only be that large if the claw is in the upward position.
//            robot.rotateRight.setPower(roughPower);
//
//            currentPosition = robot.clawDist.getDistance(DistanceUnit.MM);
//
//            telemetry.addData("Mode:", "Rough Movement");
//            telemetry.addData("Target Position:", targetPosition);
//            telemetry.addData("Current Position:", currentPosition);
//            telemetry.addData("Distance Left:", Math.abs(targetPosition-currentPosition));
//            telemetry.update();
//        }
//
//        motorStop();
//
//        sleep(waitTime);
//        telemetry.addData("Mode:", "Waiting for Fine");
//        telemetry.addData("Target Position:", targetPosition);
//        telemetry.addData("Current Position:", currentPosition);
//        telemetry.addData("Distance Left:", Math.abs(targetPosition-currentPosition));
//        telemetry.update();
//        sleep(waitTime);
//        currentPosition = robot.clawDist.getDistance(DistanceUnit.MM);
//
//
//        while (currentPosition > targetPosition){//once it gets closer, it slows down
//            robot.rotateLeft.setPower(precisePower);
//            robot.rotateRight.setPower(precisePower);
//
//            currentPosition = robot.clawDist.getDistance(DistanceUnit.MM);
//
//            telemetry.addData("Mode:", "Fine Movement");
//            telemetry.addData("Target Position:", targetPosition);
//            telemetry.addData("Current Position:", currentPosition);
//            telemetry.addData("Distance Left:", Math.abs(targetPosition-currentPosition));
//            telemetry.update();
//        }
//        motorStop();
//        sleep(waitTime);
//        telemetry.addData("Mode:", "Waiting for Correct");
//        telemetry.addData("Target Position:", targetPosition);
//        telemetry.addData("Current Position:", currentPosition);
//        telemetry.addData("Distance Left:", Math.abs(targetPosition-currentPosition));
//        telemetry.update();
//        sleep(waitTime);
//        currentPosition = robot.clawDist.getDistance(DistanceUnit.MM);
//
//        int correctionError = 10;
//        if (clawTarget == 't'||clawTarget=='h'){
//            correctionError = 0;
//        }
//
//        while (currentPosition + correctionError < targetPosition){//once it is close, it goes back up until it is within 1 cm.
//            robot.rotateLeft.setPower(-correctionSpeed);
//            robot.rotateRight.setPower(-correctionSpeed);
//
//            currentPosition = robot.clawDist.getDistance(DistanceUnit.MM);
//
//            telemetry.addData("Mode:", "Correction Movement");
//            telemetry.addData("Target Position:", targetPosition);
//            telemetry.addData("Current Position:", currentPosition);
//            telemetry.addData("Distance Left:", Math.abs(targetPosition-currentPosition));
//            telemetry.update();
//
//            if((int)currentPosition > distanceCutoff){//if it gets super high, it will (hopefully) stop.
//                break;
//            }
//        }
//        motorStop();
//        sleep(waitTime);
//        currentPosition = robot.clawDist.getDistance(DistanceUnit.MM);
//        telemetry.addData("Mode:", "Done");
//        telemetry.addData("Target Position:", targetPosition);
//        telemetry.addData("Current Position:", currentPosition);
//        telemetry.addData("Distance Left:", Math.abs(targetPosition-currentPosition));
//        telemetry.update();
//        sleep(waitTime);
//    }
}