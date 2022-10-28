/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

//@Disabled
@Autonomous(name="Red/Blue RIGHT Zone 3")

//NEED TO CORRECT THE MOVEMENT AND ORIENTATION BEFORE RUNNING. IT YEETS ITSELF FORWARD WHEN RUN
public class auton_R3 extends ppDriving {
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
    private DcMotor frontLeft   = null;
    private DcMotor frontRight   = null;
    private DcMotor backLeft   = null;
    private DcMotor backRight   = null;
    private ColorSensor colorSensor = null;


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");


        waitForStart();

//        move(.6, 'f', 5);
//        //STRAFE LEFT TO CAROUSEL, FACING TEAM WALL.
//        //move(0.3, 'f', 17);
//        //sleep(150);
//        move(.6, 'b', 5);
//
//
//        move(.6, 'l', 10);
//        move(.6, 'r', 10);
//
//        //move right
//        rotate(.5, 'l', 100);
//        rotate(.5, 'r', 100);
        //PLS NOTE THAT THIS CODE IS FOR WHEN TEH ROBOT IS FACING FORWARD AT THE START
        move(.6,'f',8);
        move(.6,'l',6);

        //color sensor code in auton example code
//        move(.6,'b',8);
//        sleep(100);
//        if (Math.abs(colorSensor.green() + (colorSensor.red()/2) - colorSensor.blue()) < 35) {
//            telemetry.addData("PURPLE FOUND",colorSensor.alpha());
//            telemetry.update();
//            move(.6,'b',5);
//        }
//        else if (colorSensor.green() > colorSensor.blue() && colorSensor.blue() > colorSensor.red()){
//            telemetry.addData("BLACK FOUND",colorSensor.alpha());
//            telemetry.update();
//            move(.6,'b',5);
//            move(.6,'r',5);
//        }
////
//        else if (Math.abs(colorSensor.blue() + (colorSensor.red()/2) - colorSensor.green()) < 30) {
//            telemetry.addData("ORANGE FOUND",colorSensor.alpha());
//            telemetry.update();
//            move(.6,'b',5);
//            move(.6,'l',5);
//        }
//        else{
//            telemetry.addData("no color found:(", colorSensor.alpha());
//            telemetry.update();
//            rotate(.6,'r',5);
//        }
//        telemetry.update();

    }
}