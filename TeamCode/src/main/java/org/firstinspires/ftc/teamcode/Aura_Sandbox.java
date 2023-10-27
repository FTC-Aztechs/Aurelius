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


import static org.firstinspires.ftc.teamcode.Aura_Robot.BUTTON_TRIGGER_TIMER_MS;
import static org.firstinspires.ftc.teamcode.Aura_Robot.bumperSpeedAdjust;
import static org.firstinspires.ftc.teamcode.Aura_Robot.dPadSpeedAdjust;
import static org.firstinspires.ftc.teamcode.Aura_Robot.speedAdjust;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.FileWriter;
import java.io.IOException;
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

@TeleOp(name="Sandbox", group="Linear Opmode")
public class Aura_Sandbox extends LinearOpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    Aura_Robot Aurelius = new Aura_Robot();
    private static ElapsedTime timer_gp1_dpad_left = new ElapsedTime();
    private static ElapsedTime timer_gp1_dpad_right = new ElapsedTime();
    boolean changingWheelSpeed = false;
    public static int SANDBOX_MODE = 0;
//    public TrajectorySequence trajPreLoadDropOff;

    enum SandboxMode
    {
        ENCODER_TESTING,   // Default - prints encoder ticks.
        VUFORIA_ESTIMATOR, // Displays current Pose from Vuforia estimate
        SEE_IT_OWN_IT,     // Enabled See it, own it claw behavior with flame extension
        SMD_LOG_MECANUMDRIVE   // Dumps motor powers from SampleMecanumDrive to file for a trajectory sequence
    }
    public static SandboxMode sandboxMode = SandboxMode.ENCODER_TESTING;

//    MvrkVuforiaPoseEstimator vuforiaPoseEstimator = new MvrkVuforiaPoseEstimator(hardwareMap);

    @Override
    public void runOpMode() {

        Aurelius.init(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Aurelius.setRunMode(Aura_Robot.AuraMotors.UPPER_LEFT, DcMotor.RunMode.STOP_AND_RESET_ENCODER); //0
        Aurelius.setRunMode(Aura_Robot.AuraMotors.LOWER_RIGHT, DcMotor.RunMode.STOP_AND_RESET_ENCODER);  //3
        Aurelius.setRunMode(Aura_Robot.AuraMotors.UPPER_RIGHT, DcMotor.RunMode.STOP_AND_RESET_ENCODER); //1

        Aurelius.setRunMode(Aura_Robot.AuraMotors.UPPER_LEFT, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Aurelius.setRunMode(Aura_Robot.AuraMotors.LOWER_RIGHT, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Aurelius.setRunMode(Aura_Robot.AuraMotors.UPPER_RIGHT, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Left Tracking wheel: ", Aurelius.getCurrentPosition(Aura_Robot.AuraMotors.UPPER_LEFT));
        telemetry.addData("Right Tracking wheel: ", Aurelius.getCurrentPosition(Aura_Robot.AuraMotors.LOWER_RIGHT));
        telemetry.addData("Strafe Tracking wheel: ", Aurelius.getCurrentPosition(Aura_Robot.AuraMotors.UPPER_RIGHT));
        // Wait for the game to start (driver presses PLAY)

//        MvrkVuforiaPoseEstimator poseEstimator = new MvrkVuforiaPoseEstimator(hardwareMap);
//        poseEstimator.setTelemetry(telemetry);

        runtime.reset();
        while(waitingForCommand() && runtime.seconds() < 4 )
        {
            telemetry.addData("Initialized, Waiting for command (4 Seconds): \n dPad ^-Encoder v-Vuforia >-SeeItOwnIt <-DriveLog ", runtime.milliseconds());
            telemetry.update();
        }

        telemetry.addData("Running in Mode: ", sandboxMode);
        telemetry.update();


        waitForStart();

        runtime.reset();

        if(sandboxMode == SandboxMode.SMD_LOG_MECANUMDRIVE ) {
//            Mavryk.MecanumDrive.enableLogging(true);
//            Mavryk.MecanumDrive.setPoseEstimate(Red_Start.pose2d());
//            Mavryk.MecanumDrive.followTrajectorySequenceAsync(trajPreLoadDropOff);
        }

        // run until the end of the match (driver presses STOP)
        while (!isStopRequested()) {
            switch(sandboxMode) {
                case SEE_IT_OWN_IT:
                   // SandboxSeeItOwnItClaw();
                    break;
                case VUFORIA_ESTIMATOR:
                    runtime.reset();
//                    poseEstimator.activateTargets(true);
                  //  SandboxVuforiaPoseEstimate();
//                    poseEstimator.activateTargets(false);
                    telemetry.addData("Time taken to get Vuforia Pose Estimate: %.3f ms",runtime.milliseconds() );
                    break;
                case SMD_LOG_MECANUMDRIVE:
                    break;
                case ENCODER_TESTING:
                    break;
                default:
                    telemetry.addData("Left Tracking wheel: ", Aurelius.getCurrentPosition(Aura_Robot.AuraMotors.UPPER_LEFT));
                    telemetry.addData("Right Tracking wheel: ", Aurelius.getCurrentPosition(Aura_Robot.AuraMotors.LOWER_RIGHT));
                    telemetry.addData("Strafe Tracking wheel: ", Aurelius.getCurrentPosition(Aura_Robot.AuraMotors.UPPER_RIGHT));

                    SandboxManualDrive();
                    break;
            }
            telemetry.update();
        }

        if(sandboxMode == SandboxMode.SMD_LOG_MECANUMDRIVE) {
            String SMDLogFilePath = String.format("%s/FIRST/data/AztechsSMDLog.txt", Environment.getExternalStorageDirectory().getAbsolutePath());
            FileWriter SMDFileWriter;
            {
                try {
                    SMDFileWriter = new FileWriter(SMDLogFilePath, false);
//                    SMDFileWriter.write(Mavryk.MecanumDrive.getSMDLogString());
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
            telemetry.addLine("Updated SMDLogFile");
        }

    }

    public boolean waitingForCommand()
    {
        if(gamepad1.dpad_up) {
            sandboxMode = SandboxMode.ENCODER_TESTING;
            return false;
        }
        else if(gamepad1.dpad_down) {
            sandboxMode = SandboxMode.VUFORIA_ESTIMATOR;
            return false;
        }
        else if(gamepad1.dpad_right) {
            sandboxMode = SandboxMode.SEE_IT_OWN_IT;
            return false;
        }
        else if(gamepad1.dpad_left) {
            sandboxMode = SandboxMode.SMD_LOG_MECANUMDRIVE;
            return false;
        }
        else
            return true;
    }

    public void SandboxManualDrive() {
        // changing the speed
        if (gamepad1.dpad_left) {
            if (!changingWheelSpeed) {
                timer_gp1_dpad_left.reset();
                changingWheelSpeed = true;
            } else if (timer_gp1_dpad_left.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
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
            } else if (timer_gp1_dpad_right.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
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
            } else if (timer_gp1_dpad_right.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {

                Aurelius.setRunMode(Aura_Robot.AuraMotors.UPPER_LEFT, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Aurelius.setRunMode(Aura_Robot.AuraMotors.LOWER_RIGHT, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Aurelius.setRunMode(Aura_Robot.AuraMotors.UPPER_RIGHT, DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                Aurelius.setRunMode(Aura_Robot.AuraMotors.UPPER_LEFT, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                Aurelius.setRunMode(Aura_Robot.AuraMotors.LOWER_RIGHT, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                Aurelius.setRunMode(Aura_Robot.AuraMotors.UPPER_RIGHT, DcMotor.RunMode.RUN_WITHOUT_ENCODER);


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
        Aurelius.lower_left.setPower((moveDir + strafeDir - turnDir) * (-speedAdjust / 10)); // 1.0
        Aurelius.lower_right.setPower((moveDir - strafeDir + turnDir) * (-speedAdjust / 10)); // 1.0
        Aurelius.upper_left.setPower((moveDir - strafeDir - turnDir) * (-speedAdjust / 10)); // 0
        Aurelius.upper_right.setPower((moveDir + strafeDir + turnDir) * (-speedAdjust / 10)); // 0

        return;
    }

}
