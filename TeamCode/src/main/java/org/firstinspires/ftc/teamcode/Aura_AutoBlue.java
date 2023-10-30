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

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Aura_Auto", group="Linear OpMode")

public class Aura_AutoBlue extends LinearOpMode {

    Aura_Robot Aurelius = new Aura_Robot();

    private static FtcDashboard auraBoard;

//TODO: declare April Tag stuff
    OpenCvWebcam Sauron = null;
    AprilTagDetectionPipeline pipeline;

    private static int iTeleCt = 1;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.04;


//TODO: Declare AprilTag IDS below
//Example: last year's custom sleeve tags

//    // Tag ID 2,9,16 from 36h11 family
//    int LEFT = 2;
//    int MIDDLE = 9;
//    int RIGHT = 20;

    public AprilTagDetection tagOfInterest = null;

//TODO: Declare Tensorflow thing below
    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

//TODO: Declare Trajectories Below
    //Example below: Powerplay trajectories

//    ElapsedTime timer = new ElapsedTime(MILLISECONDS);
//
//    //    public TrajectorySequence Park;
//    public TrajectorySequence trajPreLoadDropOff;
//    public TrajectorySequence trajCycleDropOffTopCone;
//    public TrajectorySequence trajCycleDropOffTopMidCone;
//    public TrajectorySequence trajCycleDropOffMiddleCone;
//    public TrajectorySequence trajCycleDropOffBottomMidCone;
//    public TrajectorySequence trajCycleDropOffBottomCone;
//    public TrajectorySequence trajParking;
//    public int currentCyclePickupCone = TopCone;

//TODO: declare the variable that will store the outcome for detection (already made, just uncomment)
//public static int pos = 1;



    @Override
    public void runOpMode() throws InterruptedException {
        // TODO: Assume this will be our Auto. The pseudo code below is for camera detection
        //   Option 1: Use TFOD - in this case, we simply use the ConceptTFod detector and extend it with our trained model
        //   Option 2: Develop our own OpenCV based image processor
        //               1. Implement a new VisionProcessor (kemmaProcessor)
        //                  - Implement init, processFrame and onDrawFrame methods - look at the AprilTagProcessorImpl and TfodProcesorImpl for examples.
        //                  - ProcessFrame needs to have the algorithm to detect the black (color of team element) pixels in the rectangle
        //               2. Use the VisionPortal pattern to implement camera detection (see AprilTag and tFodProcessor examples)
        //               3. Register the kemmaProcessor with VisionPortal
        //               4. Implement a method on kemmaProcessor to return detected position based on which of the 3 rectangles returns most positive
        //   Option 3: Ditch the VisionProcessor and use EasyOpenCV directly


        Aurelius.init(hardwareMap);

        ElapsedTime trajectoryTimer = new ElapsedTime(MILLISECONDS);

        auraBoard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine(String.format("%d. Aura Initialized!", iTeleCt++));

        double volts = getBatteryVoltage();
        telemetry.addLine(String.format("%d. Battery voltage: %.1f volts", iTeleCt++, volts));

        //TODO: Initialize any essential starting motor/servo positions here
//example: powerplay claw, intake, horizontal slide
//
//        Aura.setPosition(TWIN_TOWERS, Aura.Claw_Close_Pos);
//        Aura.setPosition(FUNKY_MONKEY, IntakeInsidePos);
//        Aura.setPosition(FLAMETHROWER, xSlideInPos);

        telemetry.addData("Status: ", "Building Trajectories......");
        telemetry.update();

        //TODO: Build trajectories here
//example below: build cone cycle trajectories for Powerplay
//
//        buildPreloadTrajectory();
//        if(cyclesToRun > 0)
//            trajCycleDropOffTopCone = buildCycleTrajectory(TopMidCone); // Note: Drop slides to pick up the next cone, in this case Top Mid
//        if(cyclesToRun > 1)
//            trajCycleDropOffTopMidCone = buildCycleTrajectory(MiddleCone); // Note: Drop slides to pick up the next cone, in this case Middle
//        if(cyclesToRun > 2)
//            trajCycleDropOffMiddleCone = buildCycleTrajectory(BottomMidCone); // Note: Drop slides to pick up the next cone, in this case BottomMid
//        if(cyclesToRun > 3)
//            trajCycleDropOffBottomMidCone = buildCycleTrajectory(BottomCone); // Note: Drop slides to pick up the next cone, in this case Bottom
//        if(cyclesToRun > 4)
//            trajCycleDropOffBottomCone = buildCycleTrajectory(FloorPosition); // Note: Drop slides to pick up the next cone, in this case Floor
//
//        }

        telemetry.update();

        telemetry.addData("Status: ", "Initializing camera......");
        telemetry.update();

        //TODO: Add TFOD detection here
        initTfod();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();




        while (!isStarted()){
            telemetryTfod();
        }

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetryTfod();

                // Push telemetry to the Driver Station.
                telemetry.update();

                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
//                    while(!isStarted() && !isStopRequested()) {
//                        if(x < leftbound){
//
//                        }else if(Gamex>leftbound && Gamex<rightbound){
//
//                        }else{
//
//                        }
////                    }
                }

                // Share the CPU.
                sleep(20);
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end runOpMode()


//example below: AprilTag detection Powerplay

//        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Sauron");
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        Sauron = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
//        telemetry.addData("Status: ", "camera created  ...");
//        telemetry.update();
//
//        pipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
//        telemetry.addData("Status: ", "pipeline created  ...");
//        telemetry.update();
//
//        Sauron.setPipeline(pipeline);
//        telemetry.addData("Status: ", "Pipeline set ...");
//        telemetry.update();
//
//        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
//        // out when the RC activity is in portrait. We do our actual image processing assuming
//        // landscape orientation, though.
//        Sauron.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            public void onOpened() {
//                Sauron.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
////                telemetry.addData("Status: ", "Sauron Streaming ...");
//                auraRobot.startCameraStream(Sauron, 0);
//            }
//
//            public void onError(int errorCode) {
//                return;
//            }
//        });
//
//        telemetry.addData("Status: ", "Starting April Tag detection");
//        telemetry.update();
//
//        // while waiting for the game to start, search for april tags
//        while (!isStarted() && !isStopRequested()) {
//            ArrayList<AprilTagDetection> currentDetections = pipeline.getLatestDetections();
//
//            if (currentDetections.size() != 0) {
//                boolean tagFound = false;
//
//                for (AprilTagDetection tag : currentDetections) {
//                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
//                        tagOfInterest = tag;
//                        tagFound = true;
//                        break;
//                    }
//                }
//
////                if (tagFound) {
////                    //telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
////                    //tagToTelemetry(tagOfInterest);
////                } else {
////                    telemetry.addLine("Don't see tag of interest :(");
////
////                    if (tagOfInterest == null) {
////                        //telemetry.addLine("(The tag has never been seen)");
////                    } else {
////                        //telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
////                        //tagToTelemetry(tagOfInterest);
////                    }
////                }
//
//            } else {
////                telemetry.addLine("Don't see tag of interest :(");
////
////                if (tagOfInterest == null) {
////                    telemetry.addLine("(The tag has never been seen)");
////                } else {
////                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
////                    tagToTelemetry(tagOfInterest);
////                }
//
//            }
//
////            telemetry.update();
//            sleep(20);
//        }
//
////        //once program starts
////        if (tagOfInterest != null) {
////            telemetry.addLine("Tag snapshot:\n");
////            tagToTelemetry(tagOfInterest);
////            telemetry.update();
////        } else {
////            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
////            telemetry.update();
////        }

        //TODO: Add if/else case that sets trajectory based on which position the pixel/TSE is at
//example below: April Tag switch case from Powerplay
//
//        if (tagOfInterest == null || tagOfInterest.id == LEFT) {
//            telemetry.addLine("Going to Position 1");
//            pos = 1;
//        } else if (tagOfInterest.id == MIDDLE) {
//            telemetry.addLine("Going to Position 2");
//            pos = 2;
//        } else {
//            telemetry.addLine("Going to Position 3");
//            pos = 3;
//        }
//
//        telemetry.update();

//TODO: Init any other Motors and Servos Here
//        initMotorsAndServos(true);

//TODO: Build any trajectories that depend on detection here
//example below: building park trajectory based on the TSE position
//        buildParkTrajectory(pos);
//        telemetry.update();

//TODO: Run Trajectories
//Example below: powerplay preload, cycle, and park

//        // Drop off preload
//        trajectoryTimer.reset();
//        Aura.mecanumDrive.setPoseEstimate(Blue_Start.pose2d());
//        Aura.mecanumDrive.followTrajectorySequence(trajPreLoadDropOff);
//        telemetry.addLine(String.format("%d. Preload Trajectory completed in: %.3f ", iTeleCt++, trajectoryTimer.seconds()));
//
//        // Cycle 1
//        if (cyclesToRun > 0) {
//            trajectoryTimer.reset();
//            Aura.mecanumDrive.followTrajectorySequence(trajCycleDropOffTopCone);
//            telemetry.addLine(String.format("%d. Cycle 1 Trajectory completed in: %.3f ", iTeleCt++, trajectoryTimer.seconds()));
//        }
//
//        // Cycle 2
//        if (cyclesToRun > 1) {
//            trajectoryTimer.reset();
//            Aura.mecanumDrive.followTrajectorySequence(trajCycleDropOffTopMidCone);
//            telemetry.addLine(String.format("%d. Cycle 2 Trajectory completed in: %.3f ", iTeleCt++, trajectoryTimer.seconds()));
//        }
//
//        // Cycle 3
//        if (cyclesToRun > 2) {
//            trajectoryTimer.reset();
//            Aura.mecanumDrive.followTrajectorySequence(trajCycleDropOffMiddleCone);
//            telemetry.addLine(String.format("%d. Cycle 3 Trajectory completed in: %.3f ", iTeleCt++, trajectoryTimer.seconds()));
//        }
//
//        // Cycle 4
//        if (cyclesToRun > 3) {
//            trajectoryTimer.reset();
//            Aura.mecanumDrive.followTrajectorySequence(trajCycleDropOffBottomMidCone);
//            telemetry.addLine(String.format("%d. Cycle 4 Trajectory completed in: %.3f ", iTeleCt++, trajectoryTimer.seconds()));
//        }
//
//        // Cycle 5
//        if (cyclesToRun > 4) {
//            trajectoryTimer.reset();
//            Aura.mecanumDrive.followTrajectorySequence(trajCycleDropOffBottomCone);
//            telemetry.addLine(String.format("%d. Cycle 5 Trajectory completed in: %.3f ", iTeleCt++, trajectoryTimer.seconds()));
//        }
//
//        // Park
////        EstimateCurrentPose();
////        if (!currentPose.epsilonEquals(Blue_Pickup)) {
////            Trajectory trajAdjustPos = Aura.mecanumDrive.trajectoryBuilder(currentPose)
////                    .lineToLinearHeading(Blue_Pickup)
////                    .build();
////            Aura.mecanumDrive.followTrajectory(trajAdjustPos);
////        }
//        trajectoryTimer.reset();
//        Aura.mecanumDrive.followTrajectorySequence(trajParking);
//        telemetry.addLine(String.format("%d. Park Trajectory completed in: %.3f ", iTeleCt++, trajectoryTimer.seconds()));
//
//        telemetry.update();
//    }


//TODO: Use April Tags to get current pos
//example below: pseudocode for Powerplay's vuforia

//    private void EstimateCurrentPose() {
//        //use vumarks to update current pose
//        currentPose = Blue_Pickup.pose2d();
//    }

//TODO: write trajectories as different functions
//examples below: trajectories from Powerplay

//    void buildPreloadTrajectory() {
//        telemetry.addLine(String.format("%d. buildPreloadTrajectory", iTeleCt++));
//
//        trajPreLoadDropOff = Aura.mecanumDrive.trajectorySequenceBuilder(Blue_Start.pose2d())
//                //preload
//                .lineToLinearHeading(Blue_Push_Signal.pose2d())
//                .lineToLinearHeading(Blue_Dropoff.pose2d())
//                .UNSTABLE_addTemporalMarkerOffset(PlSlideUpOffset, () -> {
//                    // Raise Tom&Jerry
//                    Aura.setTargetPosition(CAT_MOUSE, HighJunction);
//                    Aura.setRunMode(CAT_MOUSE, RUN_TO_POSITION);
//                    Aura.setPower(CAT_MOUSE, SlidePower_Up);
//                })
//                .waitSeconds(auto_raise_wait)
//                .UNSTABLE_addTemporalMarkerOffset(DropoffExtendFlamethrowerOffset, () -> { // Start after 1.5s of raise
//                    // Extend FlameThrower
//                    Aura.setPosition(FLAMETHROWER, xSlideOutPos);
//                })
//
//                .waitSeconds(auto_extend_half_wait)
//                .UNSTABLE_addTemporalMarkerOffset(PlSlideDownOffset, () -> {
//                    // Lower Tom&Jerry to Top Cone
//                    Aura.setTargetPosition(CAT_MOUSE, DropoffPos);
//                    Aura.setRunMode(CAT_MOUSE, RUN_TO_POSITION);
//                    Aura.setPower(CAT_MOUSE, SlidePower_Down);
//                    Aura.setPosition(TWIN_TOWERS, Claw_Open_Pos);
//                })
//                .waitSeconds(auto_drop_wait)
//                .addTemporalMarker(() -> {
//                    // Retract FlameThrower
//                    Aura.setPosition(FLAMETHROWER, xSlideDropPos);
//                })
//                .waitSeconds(auto_retract_wait) // Eliminate
//                .UNSTABLE_addTemporalMarkerOffset(PlSlideDownOffset, () -> {
//                    // Lower Tom&Jerry to Top Cone
//                    Aura.setTargetPosition(CAT_MOUSE, TopCone);
//                    Aura.setRunMode(CAT_MOUSE, RUN_TO_POSITION);
//                    Aura.setPower(CAT_MOUSE, SlidePower_Down);
//                })
//                .waitSeconds(auto_drop_wait)
//                .build();
//
//
//        int iNumSegments = trajPreLoadDropOff.size();
//        telemetry.addLine(String.format("%d. Preload Trajectory numTrajectory Segments: %d", iTeleCt++, iNumSegments));
//        for(int iSeg=0; iSeg<iNumSegments; iSeg++ ) {
//            telemetry.addLine(String.format("%d. Preload Trajectory Segment %d Duration: %.3f", iTeleCt++, iSeg,trajPreLoadDropOff.get(iSeg).getDuration()));
//        }
//        telemetry.addLine(String.format("%d. Park Preload calculated Duration: %.3f", iTeleCt++, trajPreLoadDropOff.duration()));
//
//        return;
//    }
//
//    void buildParkTrajectory(int iPos)
//    {
//        telemetry.addLine(String.format("%d. buildParkTrajectory", iTeleCt++));
//
//        switch (iPos) {
//            case 1:
//            default:
//                trajParking = Aura.mecanumDrive.trajectorySequenceBuilder(Blue_Dropoff.pose2d())
//                        .addTemporalMarker(()->{
//                            Aura.setPosition(FLAMETHROWER, xSlideInPos);
//                        })
//                        .lineToLinearHeading(Blue_Park_Pos1.pose2d())
//                        .build();
//                break;
//            case 2:
//                trajParking = Aura.mecanumDrive.trajectorySequenceBuilder(Blue_Dropoff.pose2d())
//                        .addTemporalMarker(()->{
//                            Aura.setPosition(FLAMETHROWER, xSlideInPos);
//                        })
//                        .lineToLinearHeading(Blue_Park_Pos2.pose2d())
//                        .build();
//                break;
//            case 3:
//                trajParking = Aura.mecanumDrive.trajectorySequenceBuilder(Blue_Dropoff.pose2d())
//                        .addTemporalMarker(()->{
//                            Aura.setPosition(FLAMETHROWER, xSlideInPos);
//                        })
//                        .lineToLinearHeading(Blue_Park_Pos3.pose2d())
//                        .build();
//                break;
//        }
//
//        int iNumSegments = trajParking.size();
//        telemetry.addLine(String.format("%d. Park Trajectory numTrajectory Segments: %d", iTeleCt++, iNumSegments));
//        for(int iSeg=0; iSeg<iNumSegments; iSeg++ ) {
//            telemetry.addLine(String.format("%d. Park Trajectory Segment %d Duration: %.3f", iTeleCt++, iSeg,trajParking.get(iSeg).getDuration()));
//        }
//        telemetry.addLine(String.format("%d. Park Trajectory calculated Duration: %.3f", iTeleCt++, trajParking.duration()));
//
//        return;
//    }
//
//    TrajectorySequence buildCycleTrajectory(int iCycleConePickup)
//    {
//        telemetry.addLine(String.format("%d. buildCycleTrajectory %d", iTeleCt++, iCycleConePickup));
//        TrajectorySequence trajSeq = Aura.mecanumDrive.trajectorySequenceBuilder(Blue_Dropoff.pose2d())
//                .lineToLinearHeading(Blue_Inter_Pos.pose2d())
//                .lineToLinearHeading(Blue_Pickup.pose2d())
//                .waitSeconds(auto_move_wait)  // Eliminate
//                .UNSTABLE_addTemporalMarkerOffset(CycleExtendFlamethrowerOffset, () -> {
//                    // Extend Flamethrower & Grab Cone
//                    Aura.setPosition(FLAMETHROWER, xSlideOutPos);
//                })
//                .waitSeconds(auto_extend_half_wait)
//                .addTemporalMarker(() -> {
//                    Aura.setPosition(TWIN_TOWERS, Claw_Close_Pos);
//                })
//                .waitSeconds(auto_pickup_wait)
//                .addTemporalMarker(() -> {
//                    // Raise to Low Junction
//                    Aura.setTargetPosition(CAT_MOUSE, LowJunction);
//                    Aura.setRunMode(CAT_MOUSE, RUN_TO_POSITION);
//                    Aura.setPower(CAT_MOUSE, SlidePower_Up);
//                })
//                .waitSeconds(auto_half_raise_wait)
//                .UNSTABLE_addTemporalMarkerOffset(CycleRetractFlamethrowerOffset, () -> {
//                    // Retract Flamethrower
//                    Aura.setPosition(FLAMETHROWER, xSlideDropPos);
//                })
//                .waitSeconds(auto_retract_wait) // Eliminate
//                .lineToLinearHeading(Blue_Dropoff.pose2d())
//                .UNSTABLE_addTemporalMarkerOffset(PlSlideUpOffset, () -> {
//                    // Raise Tom&Jerry
//                    Aura.setTargetPosition(CAT_MOUSE, HighJunction);
//                    Aura.setRunMode(CAT_MOUSE, RUN_TO_POSITION);
//                    Aura.setPower(CAT_MOUSE, SlidePower_Up);
//                })
//                .waitSeconds(auto_raise_wait)
//                .UNSTABLE_addTemporalMarkerOffset(DropoffExtendFlamethrowerOffset, () -> {
//                    // Extend FlameThrower
//                    Aura.setPosition(FLAMETHROWER, xSlideOutPos);
//                })
//                .waitSeconds(auto_extend_half_wait)
//                .UNSTABLE_addTemporalMarkerOffset(PlSlideDownOffset, () -> {
//                    // Lower Tom&Jerry to Top Cone
//                    Aura.setTargetPosition(CAT_MOUSE, DropoffPos);
//                    Aura.setRunMode(CAT_MOUSE, RUN_TO_POSITION);
//                    Aura.setPower(CAT_MOUSE, SlidePower_Down);
//                    Aura.setPosition(TWIN_TOWERS, Claw_Open_Pos);
//                })
//                .waitSeconds(auto_drop_wait)
//                .addTemporalMarker(() -> {
//                    // Retract FlameThrower
//                    Aura.setPosition(FLAMETHROWER, xSlideDropPos);
//                })
//                .waitSeconds(auto_retract_wait) // Eliminate
//                .addTemporalMarker(() -> {
//                    // Lower Tom&Jerry to Top Cone
//                    Aura.setTargetPosition(CAT_MOUSE, iCycleConePickup);
//                    Aura.setRunMode(CAT_MOUSE, RUN_TO_POSITION);
//                    Aura.setPower(CAT_MOUSE, SlidePower_Down);
//                })
//                .build();
//
//        int iNumSegments = trajSeq.size();
//        telemetry.addLine(String.format("%d. Cycle %d numTrajectory Segments: %d", iTeleCt++, iCycleConePickup, iNumSegments));
//        for(int iSeg=0; iSeg<iNumSegments; iSeg++ ) {
//            telemetry.addLine(String.format("%d. Cycle %d Trajectory Segment %d Duration: %.3f", iTeleCt++, iCycleConePickup, iSeg,trajSeq.get(iSeg).getDuration()));
//        }
//        telemetry.addLine(String.format("%d. Cycle %d Trajectory calculated Duration: %.3f", iTeleCt++, iCycleConePickup, trajSeq.duration()));
//
//        return trajSeq;
//    }

    //TODO: add any motors/servos that initialized later
    void initMotorsAndServos(boolean run_to_position)
    {
        // Reset Slides - current position becomes 0
//        Aura_Robot.setRunMode(CAT_MOUSE, STOP_AND_RESET_ENCODER);
//        Aura_Robot.setRunMode(CAT_MOUSE, RUN_WITHOUT_ENCODER);

    }

    private double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

//TODO: April Tag detection function - might need updating
//example: powerplay's initial april tag detection

//    public void getTag(AprilTagDetectionPipeline pipeline)
//    {
//        ArrayList<AprilTagDetection> currentDetections = pipeline.getLatestDetections();
//        if(currentDetections.size() != 0)
//        {
//            boolean tagFound = false;
//
//            for(AprilTagDetection tag : currentDetections)
//            {
//                if(tag.id == LEFT ||  tag.id == MIDDLE || tag.id == RIGHT)
//                {
//                    tagOfInterest = tag;
//                    tagFound = true;
//                    break;
//                }
//            }
//
//            if(tagFound)
//            {
//                telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
//                tagToTelemetry(tagOfInterest);
//            }
//            else
//            {
//                telemetry.addLine("Don't see tag of interest :(");
//
//                if(tagOfInterest == null)
//                {
//                    telemetry.addLine("(The tag has never been seen)");
//                }
//                else
//                {
//                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//                    tagToTelemetry(tagOfInterest);
//                }
//            }
//
//        }
//        else
//        {
//            telemetry.addLine("Don't see tag of interest :(");
//
//            if(tagOfInterest == null)
//            {
//                telemetry.addLine("(The tag has never been seen)");
//            }
//            else
//            {
//                telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//                tagToTelemetry(tagOfInterest);
//            }
//
//        }
//
//        telemetry.update();
//        sleep(20);
//    }
//
//    void tagToTelemetry(AprilTagDetection detection)
//    {
//        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
//        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
//        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
//        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
//        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
//        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
//        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
//    }

    //TFOD ConceptTensorFlowObjectDetectionEasy functions
    private void initTfod() {

        // Create the TensorFlow processor the easy way.
        tfod = TfodProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Kemera"), tfod);
    } // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    public void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()

}





