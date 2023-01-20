/* Copyright (c) 2021 FIRST. All rights reserved.
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

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.ppDriving;
import org.firstinspires.ftc.teamcode.ppHardware;


/**
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Tele-Slay", group="Linear Opmode")
//@Disabled
public class TELEOPslay extends ppDriving {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    ppHardware robot;
    public void setRobot(ppHardware robot){
        this.robot = robot;
    }

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        robot.frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        robot.backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        robot.frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        robot.backRight = hardwareMap.get(DcMotor.class, "backRight");
        robot.neck = hardwareMap.get(DcMotor.class, "neck");
        robot.elbow = hardwareMap.get(DcMotor.class, "elbow");
        robot.Claw = hardwareMap.get(Servo.class, "Claw");
        robot.clawRotate = hardwareMap.get(Servo.class, "clawRotate");
        robot.limit = hardwareMap.get(TouchSensor.class, "limit");

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        robot.frontLeft.setDirection(DcMotor.Direction.REVERSE);
        robot.backLeft.setDirection(DcMotor.Direction.FORWARD);
        robot.frontRight.setDirection(DcMotor.Direction.FORWARD);
        robot.backRight.setDirection(DcMotor.Direction.FORWARD);
        robot.neck.setDirection(DcMotor.Direction.FORWARD);
        robot.elbow.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double flPower = axial + lateral + yaw;
            double frPower = axial - lateral - yaw;
            double blPower = axial - lateral + yaw;
            double brPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(flPower), Math.abs(frPower));
            max = Math.max(max, Math.abs(blPower));
            max = Math.max(max, Math.abs(brPower));

            flPower /= max;
            frPower /= max;
            blPower /= max;
            brPower /= max;//hello

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.


//            flPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
//            blPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
//            frPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
//            brPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad

            // Send calculated power to wheels
            robot.frontLeft.setPower(0.4 * flPower);
            robot.frontRight.setPower(0.4 * frPower);
            robot.backLeft.setPower(0.4 * blPower);
            robot.backRight.setPower(0.4 * brPower);

            if (gamepad1.left_bumper) { //hold down left bumper for slow mode
                robot.frontLeft.setPower(0.25 * flPower);
                robot.frontRight.setPower(0.25 * frPower);
                robot.backLeft.setPower(0.25 * blPower);
                robot.backRight.setPower(0.25 * brPower);
                telemetry.addData("Touch Sensor Pressed", robot.limit.getValue());
            }
            if (gamepad1.right_bumper) { //hold down left bumper for slow mode
                robot.frontLeft.setPower(0.7 * flPower);
                robot.frontRight.setPower(0.7 * frPower);
                robot.backLeft.setPower(0.7 * blPower);
                robot.backRight.setPower(0.7 * brPower);
                telemetry.addData("Touch Sensor Pressed", robot.limit.getValue());
            }


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", flPower, frPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", blPower, brPower);

            telemetry.addData("Front left/Right", "%4.2f, %4.2f", flPower * .025, frPower * .025);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", blPower * .025, brPower * .025);
            telemetry.update();

            //2nd driver controls only the arm and claw
            // CHASSIS-BOUND HINGE CONTROL
            if (gamepad2.dpad_right) {
                robot.neck.setPower(1);// should hinge arm forward; this is just filler code, we can figure out actual numbers later
            } else {
                robot.neck.setPower(0);
            }
            if (gamepad2.dpad_left) {
                robot.neck.setPower(-1);//should hinge arm backward ; this is just filler code, we can figure out actual numbers later
            } else {
                robot.neck.setPower(0);
            }
            telemetry.addData("Touch Sensor Pressed", robot.limit.getValue());
            // ELBOW CONTROL
            if (gamepad2.dpad_up) {
                robot.elbow.setPower(0.5);//should lift arm upwards ; this is just filler code, we can figure out actual numbers later
            } else {
                robot.elbow.setPower(0);
            }
            while ((gamepad2.dpad_down)) { //robot.magStopBottom.getValue() == 0.0 //
                //elbow.setPower(-0.5);//should lower arm down; this is just filler code, we can figure out actual numbers later
                if ((robot.limit.isPressed())) {
                    telemetry.addData("GO", "YOU'RE GOING FAR DOWN!!");
                    telemetry.addData("Touch Sensor Pressed", robot.limit.getValue());
                    robot.elbow.setPower(0);
                } else {
                    robot.elbow.setPower(-0.5);
                }
            }
            //hi


            // CLAW CONTROLwqqqqq
            if (gamepad2.a) {//open ; this is just filler code, we can figure out actual numbers later -- closing in
                robot.Claw.setPosition(.75);
                telemetry.addData("claw should open", flPower);
            }
            if (gamepad2.y) {//close ; this is just filler code, we can figure out actual numbers later -- going back
                robot.Claw.setPosition(.13);
                telemetry.addData("claw should close", flPower);
            }
            if (gamepad2.b) {//rotate claw ~180 degrees; this is just filler code, we can figure out actual numbers later
                robot.clawRotate.setPosition(1);
                telemetry.addData("claw rotate", flPower);
            }
            if (gamepad2.x) {//rotate claw back to original position
                robot.clawRotate.setPosition(0.15);
                telemetry.addData("claw rotate back", flPower);
            }
            if (gamepad2.left_bumper) {
                armheight(.5);
            }
        }
        telemetry.update();
    }}
