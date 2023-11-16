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
import static org.firstinspires.ftc.teamcode.Aura_Robot.AuraMotors.ALL_DRIVES;
import static org.firstinspires.ftc.teamcode.Aura_Robot.AuraMotors.LOWER_LEFT;
import static org.firstinspires.ftc.teamcode.Aura_Robot.AuraMotors.LOWER_RIGHT;
import static org.firstinspires.ftc.teamcode.Aura_Robot.AuraMotors.UPPER_LEFT;
import static org.firstinspires.ftc.teamcode.Aura_Robot.AuraMotors.UPPER_RIGHT;
import static org.firstinspires.ftc.teamcode.Aura_Robot.BUTTON_TRIGGER_TIMER_MS;
import static org.firstinspires.ftc.teamcode.Aura_Robot.bumperSpeedAdjust;
import static org.firstinspires.ftc.teamcode.Aura_Robot.dPadSpeedAdjust;
import static org.firstinspires.ftc.teamcode.Aura_Robot.speedAdjust;
import static java.util.concurrent.TimeUnit.MILLISECONDS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
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

    public static int Smd_profileTime = 5000;

    // HTML Logging
    private File logFile;
    private PrintWriter printWriter;

    ArrayList<String> htmlLog = new ArrayList<>();

    enum SandboxMode
    {
        ENCODER_TESTING,   // Default - prints encoder ticks.
        SMD_WHEEL_MOTOR_PROFILER,
        SMD_INTAKE_OUTTAKE
    }
    public static SandboxMode sandboxMode = SandboxMode.SMD_WHEEL_MOTOR_PROFILER;

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


        telemetry.addData("Right Tracking wheel: ", Aurelius.getCurrentPosition(LOWER_LEFT));
        telemetry.addData("Left Tracking wheel: ", Aurelius.getCurrentPosition(UPPER_RIGHT));
        telemetry.addData("Strafe Tracking wheel: ", Aurelius.getCurrentPosition(LOWER_RIGHT));

        waitForStart();
        //getUserInput();

        telemetry.addData("Running in Mode: ", sandboxMode);
        telemetry.update();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (!isStopRequested()) {
            switch(sandboxMode) {
                case SMD_INTAKE_OUTTAKE:
                    SandboxIntakeOuttake();
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
                    telemetry.addData("Right Tracking wheel: ", Aurelius.getCurrentPosition(LOWER_LEFT));
                    telemetry.addData("Left Tracking wheel: ", Aurelius.getCurrentPosition(UPPER_RIGHT));
                    telemetry.addData("Strafe Tracking wheel: ", Aurelius.getCurrentPosition(LOWER_RIGHT));
                    SandboxManualDrive();
                    break;
            }
            telemetry.update();
        }
    }

    public void SandboxMotorProfile(Aura_Robot.AuraMotors eWhichMotor) {

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

        Aurelius.setPower(Aura_Robot.AuraMotors.INTAKE, (dPadSpeedAdjust/10)*gamepad2.right_stick_y);
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
                Aurelius.setRunMode(Aura_Robot.AuraMotors.LOWER_RIGHT, STOP_AND_RESET_ENCODER);
                Aurelius.setRunMode(UPPER_RIGHT, STOP_AND_RESET_ENCODER);

                Aurelius.setRunMode(UPPER_LEFT, RUN_WITHOUT_ENCODER);
                Aurelius.setRunMode(Aura_Robot.AuraMotors.LOWER_RIGHT, RUN_WITHOUT_ENCODER);
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
