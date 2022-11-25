package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.opencv.objdetect.QRCodeDetector;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@TeleOp

public class ViewTest extends LinearOpMode {
    OpenCvWebcam webcam;
    QRCodeDetector det;
    boolean found;
    String message;
    private String finalMessage = "";
    private boolean hasStarted = false;
    private boolean isStopped = false;
    ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontLeft   = null;
    private DcMotor frontRight   = null;
    private DcMotor backLeft   = null;
    private DcMotor backRight   = null;
    //public DcMotor liftLeft = null;
    //public DcMotor liftRight = null;
    //private ColorSensor colorSensor = null;

    @Override
    public void runOpMode() {

        //robot.init(hardwareMap);

        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
//        liftLeft = hardwareMap.get(DcMotor.class,"liftLeft");
//        liftRight = hardwareMap.get(DcMotor.class,"liftRight");
//        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");



        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(new SimplePipeline());
        det = new QRCodeDetector();
        /*
         * Open the connection to the camera device. New in v1.4.0 is the ability
         * to open the camera asynchronously, and this is now the recommended way
         * to do it. The benefits of opening async include faster init time, and
         * better behavior when pressing stop during init (i.e. less of a chance
         * of tripping the stuck watchdog)
         *
         * If you really want to open synchronously, the old method is still available.
         */
        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // telemetry.addData("Frame Count", webcam.getFrameCount());
            // telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            // telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            // telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            // telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            // telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            // telemetry.update();
            // // telemetry.addData("Status", "Running");
            // telemetry.update();


                hasStarted = true;
                runtime.reset();
                telemetry.addData("Message: ", message);
                telemetry.update();
                sleep(1000);

                try {
                    switch (message) {
                        case "https://left.com":
                            // strafe left one tile

                            break;
                        case "https://middle.com":
                            // no need to move
                            break;
                        case "https://right.com":
                            // strafe right one tile

                            break;
                        default:
                            // hope it's middle, attempting to recheck
                            throw new NullPointerException();
                    }
                } catch(NullPointerException e){
                    telemetry.addData("Message: ", "QR Code not found. Trying again. ");

                    try {
                        switch (message) {
                            case "https://left.com":
                                // strafe left one tile

                                break;
                            case "https://middle.com":
                                // no need to move
                                break;
                            case "https://right.com":
                                // strafe right one tile

                                break;
                            default:
                                // hope it's middle, attempting to recheck
                                throw new NullPointerException();
                        }
                    }catch(NullPointerException exception){
                        telemetry.addData("Message: ", "QR Code still not found. Giving up. ");
                    }
                }
            }

        }

    }

    public class SimplePipeline extends OpenCvPipeline{
        public Mat processFrame(Mat input){
            String decoded = det.detectAndDecode(input);
            if (decoded.length() > 5){
                telemetry.addData("Decoded",decoded);
                telemetry.update();
                sleep(10000);
                found = true;
                message = decoded;
            }

            return input;
        }

    }
