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


import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static org.firstinspires.ftc.teamcode.AuraRobot.AuraMotors.ALL_DRIVES;
import static org.firstinspires.ftc.teamcode.AuraRobot.AuraMotors.LOWER_LEFT;
import static org.firstinspires.ftc.teamcode.AuraRobot.AuraMotors.LOWER_RIGHT;
import static org.firstinspires.ftc.teamcode.AuraRobot.AuraMotors.UPPER_LEFT;
import static org.firstinspires.ftc.teamcode.AuraRobot.AuraMotors.UPPER_RIGHT;
import static org.firstinspires.ftc.teamcode.AuraRobot.BUTTON_TRIGGER_TIMER_MS;
import static org.firstinspires.ftc.teamcode.AuraRobot.bumperSpeedAdjust;
import static org.firstinspires.ftc.teamcode.AuraRobot.dPadSpeedAdjust;
import static org.firstinspires.ftc.teamcode.AuraRobot.speedAdjust;
import static java.util.concurrent.TimeUnit.MILLISECONDS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Config
@TeleOp(name="Sandbox", group="Linear Opmode")
public class Aura_Sandbox extends LinearOpMode
{
    // Declare OpMode members.
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = 3;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    public static boolean CAMERA_SIDE = false;
    private ElapsedTime runtime = new ElapsedTime();
    AuraRobot Aurelius = new AuraRobot();
    private static ElapsedTime timer_gp1_dpad_left = new ElapsedTime();
    private static ElapsedTime timer_gp1_dpad_right = new ElapsedTime();
    private static ElapsedTime timer_gp2_x = new ElapsedTime();
    private static ElapsedTime timer_gp2_y = new ElapsedTime();
    private static ElapsedTime timer_gp2_a = new ElapsedTime();

    boolean changingWheelSpeed = false;
    boolean targetFound = false;    // Set to true when an AprilTag target is detected
    boolean changingState = false;
    boolean bKeepGoing = true;
    public static int SANDBOX_MODE = 0;

    public static int Smd_profileTime = 5000;

    // HTML Logging
    private File logFile;
    private PrintWriter printWriter;

    ArrayList<String> htmlLog = new ArrayList<>();

    enum SandboxMode
    {
        ENCODER_TESTING,   // Default - prints encoder ticks.
        SMD_WHEEL_MOTOR_PROFILER,
        SMD_INTAKE_OUTTAKE,
        HANG,
        APRIL
    }
    public static SandboxMode sandboxMode = SandboxMode.APRIL;

//    public static class CameraStreamProcessor implements VisionProcessor, CameraStreamSource {
//        private final AtomicReference<Bitmap> lastFrame =
//                new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));
//
//        @Override
//        public void init(int width, int height, CameraCalibration calibration) {
//            lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
//        }
//
//        @Override
//        public Object processFrame(Mat frame, long captureTimeNanos) {
//            Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
//            Utils.matToBitmap(frame, b);
//            lastFrame.set(b);
//            return null;
//        }
//
//        @Override
//        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
//                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
//                                Object userContext) {
//            // do nothing
//        }
//
//        @Override
//        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
//            continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
//        }
//    }

//    MvrkVuforiaPoseEstimator vuforiaPoseEstimator = new MvrkVuforiaPoseEstimator(hardwareMap);

    @Override
    public void runOpMode() {
        Aurelius.init(hardwareMap);

        // Telemetry and HTML Log file
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Aurelius.setRunMode(LOWER_LEFT, STOP_AND_RESET_ENCODER); //3
        Aurelius.setRunMode(UPPER_RIGHT, STOP_AND_RESET_ENCODER);  //0
        Aurelius.setRunMode(LOWER_RIGHT, STOP_AND_RESET_ENCODER); //1

        Aurelius.setRunMode(LOWER_LEFT, RUN_WITHOUT_ENCODER);
        Aurelius.setRunMode(UPPER_RIGHT, RUN_WITHOUT_ENCODER);
        Aurelius.setRunMode(LOWER_RIGHT, RUN_WITHOUT_ENCODER);


        telemetry.addData("Left Tracking wheel: ", Aurelius.getCurrentPosition(LOWER_LEFT));
        telemetry.addData("Right Tracking wheel: ", Aurelius.getCurrentPosition(UPPER_RIGHT));
        telemetry.addData("Strafe Tracking wheel: ", Aurelius.getCurrentPosition(LOWER_RIGHT));
        initAprilTag();
        if(USE_WEBCAM) {
            setManualExposure(6,250);
        }

        waitForStart();
        //getUserInput();

        telemetry.addData("Running in Mode: ", sandboxMode);
        telemetry.update();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (!isStopRequested() && bKeepGoing) {
            switch(sandboxMode) {
                case SMD_INTAKE_OUTTAKE:
                    SandboxIntakeOuttake();
                    break;
                case HANG:
                    HangSandbox();
                    break;
                case APRIL:
                    AprilSandbox();
                    break;
                case SMD_WHEEL_MOTOR_PROFILER:
                    logFile = new File("/sdcard/FIRST/www/SandboxTelemetry.html");
                    try {
                        // create the www folder if it doesn't exist
                        new File("/sdcard/FIRST/www/").mkdirs();
                        printWriter = new PrintWriter(new FileWriter(logFile, false));
                    }catch (IOException e) {
                        telemetry.addData("Error", "Failed to create log file");
                        telemetry.update();
                        sleep(2000);
                        requestOpModeStop();
                    }
                    SandboxMotorProfile(UPPER_LEFT);
                    logTelemetryToHTML();
                    if(printWriter != null)
                        printWriter.close();
                break;
                case ENCODER_TESTING:
                default:
//                    telemetry.addData("Right Tracking wheel: ", Aurelius.getCurrentPosition(LOWER_LEFT));
//                    telemetry.addData("Left Tracking wheel: ", Aurelius.getCurrentPosition(UPPER_RIGHT));
//                    telemetry.addData("Strafe Tracking wheel: ", Aurelius.getCurrentPosition(LOWER_RIGHT));
                    SandboxManualDrive();
                    break;
            }
            telemetry.addData("Left Tracking wheel: ", -Aurelius.getCurrentPosition(LOWER_LEFT));
            telemetry.addData("Right Tracking wheel: ", Aurelius.getCurrentPosition(UPPER_RIGHT));
            telemetry.addData("Strafe Tracking wheel: ", Aurelius.getCurrentPosition(LOWER_RIGHT));
            telemetry.update();
        }
    }

    public void SandboxMotorProfile(AuraRobot.AuraMotors eWhichMotor) {

        // Connect motor and encoder cable to port 1 of Robot Controller
        // Reset encoder.
        // Run motor at 0.1 speed for 1 second - write out encoder ticks
        // Run motor at 0.5 speed for 1 second - write out encoder ticks
        // Run motor at 1.0 speed for 1 second - write out encoder ticks
        // Run motor at speeds accelerating along a sine curve for 3 seconds - write out encoder ticks
        ElapsedTime timer = new ElapsedTime();
        Aurelius.setRunMode(ALL_DRIVES, RUN_WITHOUT_ENCODER);
        Aurelius.setRunMode(ALL_DRIVES, STOP_AND_RESET_ENCODER);
        timer.reset();
        while(timer.time(MILLISECONDS) < Smd_profileTime)
        {
            if(timer.time(MILLISECONDS) < (Smd_profileTime/2))
                Aurelius.setPower(eWhichMotor,.1);
            else
                Aurelius.setPower(eWhichMotor, -0.1);
        }
        htmlLog.add(telemetry.addData("0.1 speed test: # Encoder Ticks Upper_Left  = ", Aurelius.getCurrentPosition(UPPER_LEFT)).toString());
        htmlLog.add(telemetry.addData("0.1 speed test: # Encoder Ticks Upper_Right = ", Aurelius.getCurrentPosition(UPPER_RIGHT)).toString());
        htmlLog.add( telemetry.addData("0.1 speed test: # Encoder Ticks Lower_Left  = ", Aurelius.getCurrentPosition(LOWER_LEFT)).toString());
        htmlLog.add(telemetry.addData("0.1 speed test: # Encoder Ticks Lower_Right = ", Aurelius.getCurrentPosition(LOWER_RIGHT)).toString());
        telemetry.update();

        Aurelius.setRunMode(ALL_DRIVES, STOP_AND_RESET_ENCODER);
        timer.reset();
        while(timer.time(MILLISECONDS) < Smd_profileTime)
        {
            if(timer.time(MILLISECONDS) < (Smd_profileTime/2))
                Aurelius.setPower(eWhichMotor,.5);
            else
                Aurelius.setPower(eWhichMotor, -0.5);
        }
        htmlLog.add(telemetry.addData("0.5 speed test: # Encoder Ticks Upper_Left  = ", Aurelius.getCurrentPosition(UPPER_LEFT)).toString());
        htmlLog.add(telemetry.addData("0.5 speed test: # Encoder Ticks Upper_Right = ", Aurelius.getCurrentPosition(UPPER_RIGHT)).toString());
        htmlLog.add(telemetry.addData("0.5 speed test: # Encoder Ticks Lower_Left  = ", Aurelius.getCurrentPosition(LOWER_LEFT)).toString());
        htmlLog.add(telemetry.addData("0.5 speed test: # Encoder Ticks Lower_Right = ", Aurelius.getCurrentPosition(LOWER_RIGHT)).toString());
        telemetry.update();

        Aurelius.setRunMode(ALL_DRIVES, STOP_AND_RESET_ENCODER);
        timer.reset();
        while(timer.time(MILLISECONDS) < Smd_profileTime)
        {
            if(timer.time(MILLISECONDS) < (Smd_profileTime/2))
                Aurelius.setPower(eWhichMotor,1);
            else
                Aurelius.setPower(eWhichMotor, -1);
        }
        htmlLog.add(telemetry.addData("1 speed test: # Encoder Ticks Upper_Left  = ", Aurelius.getCurrentPosition(UPPER_LEFT)).toString());
        htmlLog.add(telemetry.addData("1 speed test: # Encoder Ticks Upper_Right = ", Aurelius.getCurrentPosition(UPPER_RIGHT)).toString());
        htmlLog.add(telemetry.addData("1 speed test: # Encoder Ticks Lower_Left  = ", Aurelius.getCurrentPosition(LOWER_LEFT)).toString());
        htmlLog.add(telemetry.addData("1 speed test: # Encoder Ticks Lower_Right = ", Aurelius.getCurrentPosition(LOWER_RIGHT)).toString());
        telemetry.update();

        Aurelius.setRunMode(ALL_DRIVES, STOP_AND_RESET_ENCODER);
        timer.reset();
        while(timer.time(MILLISECONDS) < Smd_profileTime)
        {
            double angle = 2*Math.PI * timer.time(TimeUnit.SECONDS) / (Smd_profileTime/1000);
            double sinePower = Math.sin(angle);
            Aurelius.setPower(eWhichMotor,sinePower);
        }
        htmlLog.add(telemetry.addData("ramp speed test: # Encoder Ticks Upper_Left  = ", Aurelius.getCurrentPosition(UPPER_LEFT)).toString());
        htmlLog.add(telemetry.addData("ramp speed test: # Encoder Ticks Upper_Right = ", Aurelius.getCurrentPosition(UPPER_RIGHT)).toString());
        htmlLog.add(telemetry.addData("ramp speed test: # Encoder Ticks Lower_Left  = ", Aurelius.getCurrentPosition(LOWER_LEFT)).toString());
        htmlLog.add(telemetry.addData("ramp speed test: # Encoder Ticks Lower_Right = ", Aurelius.getCurrentPosition(LOWER_RIGHT)).toString());
        telemetry.update();
    }

    void SandboxIntakeOuttake()
    {
        if (gamepad2.dpad_left) {
            if (!changingWheelSpeed) {
                timer_gp1_dpad_left.reset();
                changingWheelSpeed = true;
            } else if (timer_gp1_dpad_left.time(MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                if (dPadSpeedAdjust <= 1) {
                    dPadSpeedAdjust = 1;
                } else {
                    dPadSpeedAdjust -= 1;
                }
                telemetry.addLine("Current speed: " + dPadSpeedAdjust);
                telemetry.update();
                changingWheelSpeed = false;
            }
        }

        //gamepad right -> increase wheel speed
        if (gamepad2.dpad_right) {
            if (!changingWheelSpeed) {
                timer_gp1_dpad_right.reset();
                changingWheelSpeed = true;
            } else if (timer_gp1_dpad_right.time(MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                if (dPadSpeedAdjust >= 10) {
                    dPadSpeedAdjust = 10;
                } else {
                    dPadSpeedAdjust += 1;
                }
                telemetry.addLine("Current speed: " + dPadSpeedAdjust);
                telemetry.update();
                changingWheelSpeed = false;
            }
        }

        Aurelius.setPower(AuraRobot.AuraMotors.INTAKE, (dPadSpeedAdjust/10)*gamepad2.right_stick_y);
    }

    public void getUserInput()
    {
        telemetry.addData("Please select command (GP1): ", "a for Encoder Testing, b for Wheel Profiles, x for IntakeOutTake");
        if(gamepad1.a) {
            sandboxMode = SandboxMode.ENCODER_TESTING;
        }
        else if(gamepad1.b) {
            sandboxMode = SandboxMode.SMD_WHEEL_MOTOR_PROFILER;
        }
        else if(gamepad1.x) {
            sandboxMode = SandboxMode.SMD_INTAKE_OUTTAKE;
        }
        idle();
    }

    public void SandboxManualDrive() {
        // changing the speed
        if (gamepad1.dpad_left) {
            if (!changingWheelSpeed) {
                timer_gp1_dpad_left.reset();
                changingWheelSpeed = true;
            } else if (timer_gp1_dpad_left.time(MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                if (dPadSpeedAdjust <= 1) {
                    dPadSpeedAdjust = 1;
                } else {
                    dPadSpeedAdjust -= 1;
                }
                telemetry.addLine("Current speed: " + dPadSpeedAdjust);
                telemetry.update();
                changingWheelSpeed = false;
            }
        }

        //gamepad right -> increase wheel speed
        if (gamepad1.dpad_right) {
            if (!changingWheelSpeed) {
                timer_gp1_dpad_right.reset();
                changingWheelSpeed = true;
            } else if (timer_gp1_dpad_right.time(MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                if (dPadSpeedAdjust >= 10) {
                    dPadSpeedAdjust = 10;
                } else {
                    dPadSpeedAdjust += 1;
                }
                telemetry.addLine("Current speed: " + dPadSpeedAdjust);
                telemetry.update();
                changingWheelSpeed = false;
            }
        }

        if(gamepad1.a) {
            if (!changingWheelSpeed) {
                timer_gp1_dpad_right.reset();
                changingWheelSpeed = true;
            } else if (timer_gp1_dpad_right.time(MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {

                Aurelius.setRunMode(UPPER_LEFT, STOP_AND_RESET_ENCODER);
                Aurelius.setRunMode(AuraRobot.AuraMotors.LOWER_RIGHT, STOP_AND_RESET_ENCODER);
                Aurelius.setRunMode(UPPER_RIGHT, STOP_AND_RESET_ENCODER);

                Aurelius.setRunMode(UPPER_LEFT, RUN_WITHOUT_ENCODER);
                Aurelius.setRunMode(AuraRobot.AuraMotors.LOWER_RIGHT, RUN_WITHOUT_ENCODER);
                Aurelius.setRunMode(UPPER_RIGHT, RUN_WITHOUT_ENCODER);


                telemetry.addLine("Reset Encoders");
                telemetry.update();
                changingWheelSpeed = false;
            }
        }

        //bumper speed boost mode
        if (gamepad1.right_bumper) {
            speedAdjust = bumperSpeedAdjust;
        } else {
            speedAdjust = dPadSpeedAdjust;
        }

        // actually making the robot move
        float turnDir = gamepad1.right_stick_x;
        float moveDir = gamepad1.left_stick_y;
        float strafeDir = gamepad1.left_stick_x;

        if (turnDir > 1) {
            turnDir = 1;
        } else if (turnDir < -1) {
            turnDir = -1;
        }
        Aurelius.Lower_Left.setPower((moveDir + strafeDir - turnDir) * (-speedAdjust / 10)); // 1.0
        Aurelius.Lower_Right.setPower((moveDir - strafeDir + turnDir) * (-speedAdjust / 10)); // 1.0
        Aurelius.Upper_Left.setPower((moveDir - strafeDir - turnDir) * (-speedAdjust / 10)); // 0
        Aurelius.Upper_Right.setPower((moveDir + strafeDir + turnDir) * (-speedAdjust / 10)); // 0

        return;
    }

    public void AprilSandbox() {

        targetFound = false;
        desiredTag  = null;

        int desiredTagID = CAMERA_SIDE ? 3 : 4;

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((DESIRED_TAG_ID < 0) || (detection.id == desiredTagID)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }

        if (targetFound) {
            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.fieldPosition);
            telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
            bKeepGoing = false;
        }
        else {
            telemetry.addData("Target:", "Not Found!");
        }
        telemetry.update();

        //range 40
        //bearing 17
        //yaw 0
    }

    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            if(CAMERA_SIDE) {
                visionPortal = new VisionPortal.Builder()
                        .setCamera(hardwareMap.get(WebcamName.class, "Kemera"))
                        .addProcessor(aprilTag)
                        .build();
            } else  {
                visionPortal = new VisionPortal.Builder()
                        .setCamera(hardwareMap.get(WebcamName.class, "Eyeball"))
                        .addProcessor(aprilTag)
                        .build();
            }
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

    public void HangSandbox() {
        if(gamepad2.x) {
            if (!changingState) {
                timer_gp2_x.reset();
                changingState = true;
            } else if (timer_gp2_x.time(MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                Aurelius.hanger.setTargetState(AuraHangController.HangState.Idle);
                telemetry.addData("State ", " Idle");
                changingState = false;
            }
        }
        if (gamepad2.y) {
            if (!changingState) {
                timer_gp2_y.reset();
                changingState = true;
            } else if (timer_gp2_y.time(MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                Aurelius.hanger.setTargetState(AuraHangController.HangState.Up);
                telemetry.addData("State ", " Up");
                changingState = false;
            }
        }
        if (gamepad2.a) {
            if (!changingState) {
                timer_gp2_a.reset();
                changingState = true;
            } else if (timer_gp2_a.time(MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                Aurelius.hanger.setTargetState(AuraHangController.HangState.Hang);
                telemetry.addData("State ", " Hang");

                changingState = false;
            }
        }
        Aurelius.hanger.update();
    }

    private void logTelemetryToHTML() {
        if(printWriter != null) {
            // Write HTML header
            printWriter.println("<html><head><title>Telemetry Log</title></head><body>");

            // Write telemetry data
            printWriter.println("<h2>Sandbox Telemetry Data</h2>");
            printWriter.println("<pre>");
            for (String line : htmlLog) {
                printWriter.println("<p>" + line + "</p>");
            }
            printWriter.println("</body></html>");
            printWriter.flush();
        }
    }

}
