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
 * SERVICES; LOSS OF USE, Line, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;
import static org.firstinspires.ftc.teamcode.Aura_Robot.BUTTON_TRIGGER_TIMER_MS;
import static org.firstinspires.ftc.teamcode.Aura_Robot.bumperSpeedAdjust;
import static org.firstinspires.ftc.teamcode.Aura_Robot.dPadSpeedAdjust;
import static org.firstinspires.ftc.teamcode.Aura_Robot.speedAdjust;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;


@Config
@TeleOp(name="Aura_Manual", group="Manual mode")
//@Disabled
public class Aura_Manual extends LinearOpMode {

    // Declare OpMode members.
    Aura_Robot Aurelius = new Aura_Robot();

    private boolean changingWheelSpeed = false; //truer than false;

    private int slide_currentPos = 0;
    private int slide_newPos = slide_currentPos;


    public static int Mode = 1;
//    public static int BUTTON_TRIGGER_TIMER_MS = 500;

    boolean ServoTurn = false;

    //    private static ElapsedTime timer_gp1_buttonA;
//    private static ElapsedTime timer_gp1_buttonX;
//    private static ElapsedTime timer_gp1_buttonY;
//    private static ElapsedTime timer_gp1_buttonB;
//    private static ElapsedTime timer_gp1_dpad_up;
//    private static ElapsedTime timer_gp1_dpad_down;
    private static ElapsedTime timer_gp2_buttonA = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp2_buttonX = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp2_buttonY = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp2_buttonB = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp1_dpad_left = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp1_dpad_right = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp2_dpad_up = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp2_dpad_down = new ElapsedTime(MILLISECONDS);
    //    private static ElapsedTime timer_gp2_buttonA = new ElapsedTime(MILLISECONDS);
//    private static ElapsedTime timer_gp2_buttonX = new ElapsedTime(MILLISECONDS);
//    private static ElapsedTime timer_gp2_buttonY = new ElapsedTime(MILLISECONDS);
//    private static ElapsedTime timer_gp2_buttonB = new ElapsedTime(MILLISECONDS);
//    ;
//    private static ElapsedTime timer_gp2_dpad_up = new ElapsedTime(MILLISECONDS);
//    private static ElapsedTime timer_gp2_dpad_down = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp2_dpad_left = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp2_dpad_right = new ElapsedTime(MILLISECONDS);

    private static ElapsedTime timer_gp1_left_bumper = new ElapsedTime(MILLISECONDS);

    //slide button booleans
    private boolean assumingHighPosition = false;
    private boolean assumingMidPosition = false;
    private boolean assumingLowPosition = false;
    private boolean assumingFloorPosition = false;
    private boolean assumingTopMidCone = false;
    private boolean assumingMiddleCone = false;


    //drive booleans
    private boolean changing_drive_mode = false;
    private boolean fieldCentric = false;


    FtcDashboard auraDashboard;


    @Override
    public void runOpMode() {
//        PhotonCore.enable();
        // Initialize the drive system vriables
        Aurelius.init(hardwareMap);

        initAurelius();
        waitForStart();


        while (opModeIsActive()) {
//            AuraClaw();
//            AuraXSlide();
//            AuraUpSlide_Pid();
//            MrvkTurret();

//            if(gamepad1.left_bumper) {
//                if (!changing_drive_mode) {
//                    timer_gp1_left_bumper.reset();
//                    changing_drive_mode = true;
//                } else if (timer_gp1_left_bumper.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
//                    fieldCentric = !fieldCentric;
//                    changing_drive_mode = false;
//                }
        }

//            if(fieldCentric){
//                AuraManualDrive_FieldCentric();
//                telemetry.addLine("Drive Mode: Field Centric");
//                telemetry.update();
//            }else{
        AuraManualDrive();
        telemetry.addLine("Drive Mode: Forward Facing");
        telemetry.update();
//            }
    }
//    }

    public void initAurelius() {
        msStuckDetectStop = 2500;
        FtcDashboard Dash = auraDashboard;

//        Aurelius.setRunMode(Aura_Robot.AuraMotors.CAT_MOUSE, STOP_AND_RESET_ENCODER);
//        Aurelius.setRunMode(Aura_Robot.AuraMotors.CAT_MOUSE, RUN_WITHOUT_ENCODER);

//        Claw_Position = Claw_Close_Pos;
//        Aurelius.Looney.setPosition(Claw_Position);

//        xSlide_Position = xSlideInPos;
//        Aurelius.FlameThrower.setPosition(xSlide_Position);

//        Aurelius.Teacup.setPosition(turretUp);
//        telemetry.addData("angle", Aurelius.Teacup.getPosition());

//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
//        BNO055IMUUtil.remapZAxis(imu, AxisDirection.NEG_Y);
//        imu.initialize(parameters);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Status: Robot is ready to roll!");
        telemetry.update();

        return;
    }

    public void AuraManualDrive() {
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



//    public void AuraManualDrive_FieldCentric()
//    {
//
//        Localizer myLocaliizer = Aurelius.MecanumDrive.getLocalizer();
//        myLocaliizer.getPoseVelocity();
//        Vector2d input = new Vector2d( -gamepad1.left_stick_y,
//                -gamepad1.left_stick_x).rotated(-imu.getAngularOrientation().firstAngle);
//
//        Aurelius.MecanumDrive.setWeightedDrivePower(new Pose2d(input.getX(), input.getY(), -gamepad1.right_stick_x));
//        Aurelius.MecanumDrive.update();
//
//        return;
//    }


//    public void AuraClaw() {
//        ServoTurn = gamepad2.right_trigger == 1f;
//
//
//        if (ServoTurn) {
//            Claw_Position = Claw_Open_Pos;
//        } else {
//            Claw_Position = Claw_Close_Pos;
//        }
//
//        Aurelius.setPosition(Aura_Robot.AuraServos.CARTOON, Claw_Position);
//
//    }

//    public void AuraUpSlide_Pid() {
//        //restrict range and provide warnings
//        slideHeightMinExtension = LowerLimit;
//        slideHeightMaxExtension = HighJunction;
//
//        //joystick control
//        slide_newPos += (int) (-gamepad2.left_stick_y * slideTicks_stepSize);
//
//        //button control
//        if (gamepad2.y) {
//            if (!assumingHighPosition) {
//                timer_gp2_buttonY.reset();
//                assumingHighPosition = true;
//            } else if (timer_gp2_buttonY.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
//                telemetry.addLine("GP2_Y triggered. Set Tom&Jerry to High position");
//                telemetry.update();
//                slide_newPos = HighJunction;
//                assumingHighPosition = false;
//                if(HighJunction >= slideHeightMinExtension) {
//                } else {
//                    telemetry.addData("Vertical Slides", "Move to High Junction locked. Turn the turret forward to unlock full range!");
//                    telemetry.update();
//                    assumingHighPosition = false;
//                }
//            }
//        }
//
//        if (gamepad2.x) {
//            if (!assumingMidPosition) {
//                timer_gp2_buttonX.reset();
//                assumingMidPosition = true;
//            } else if (timer_gp2_buttonX.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
//
//                if( MidJunction >= slideHeightMinExtension) {
//                    telemetry.addLine("GP2_X triggered. Set Tom&Jerry to Mid position");
//                    telemetry.update();
//                    slide_newPos = MidJunction;
//                    assumingHighPosition = false;
//                } else {
//                    telemetry.addData("Vertical Slides", "Move to MidJunction is locked. Turn the turret forward to unlock full range!");
//                    telemetry.update();
//                    assumingMidPosition = false;
//                }
//            }
//        }
//
//        if(gamepad2.a) {
//            if (!assumingLowPosition) {
//                timer_gp2_buttonY.reset();
//                assumingLowPosition = true;
//            } else if (timer_gp2_buttonA.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
//
//                if(LowJunction >= slideHeightMinExtension) {
//                    telemetry.addLine("GP2_A triggered. Set Tom&Jerry to Low position");
//                    telemetry.update();
//                    slide_newPos = LowJunction;
//                    assumingLowPosition = false;
//                } else {
//                    telemetry.addData("Vertical Slides", "Move to Low Junction is locked. Turn the turret forward to unlock full range!");
//                    telemetry.update();
//                    assumingLowPosition = false;
//                }
//
//
//            }
//        }
//
//        if (gamepad2.b) {
//            if (!assumingFloorPosition) {
//                timer_gp2_buttonB.reset();
//                assumingFloorPosition = true;
//            } else if (timer_gp2_buttonB.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
//
//                if( LowerLimit >= slideHeightMinExtension) {
//                    telemetry.addLine("GP2_B triggered. Set Tom&Jerry to Floor position");
//                    telemetry.update();
//                    slide_newPos = FloorPosition;
//                    assumingFloorPosition = false;
//                } else {
//                    telemetry.addData("Vertical Slides", "Move to Floor Position is locked. Turn the turret forward to unlock full range!");
//                    telemetry.update();
//                    assumingFloorPosition = false;
//                }
//            }
//        }
//
//        telemetry.addData("newPos", slide_newPos);
//        telemetry.update();
//
//        //capping control
//        if(slide_newPos >= slideHeightMaxExtension)
//            slide_newPos = slideHeightMaxExtension;
//        else if (slide_newPos <= slideHeightMinExtension)
//            slide_newPos = slideHeightMinExtension;
//        telemetry.addData("newPos", slide_newPos);
//        telemetry.update();
//
//        //actually using the pid to move the slides
//        if( slide_newPos != slide_currentPos && slide_newPos >= slideHeightMinExtension && slide_newPos <= slideHeightMaxExtension ) {
//            double command = manualSlidePID.output(slide_newPos, Aurelius.getCurrentPosition(Aura_Robot.AuraMotors.CAT_MOUSE));
//            if(slide_newPos < slide_currentPos)
//                SlidePower = Math.max(command/HighJunction, SlidePower_Down);
//            else
//                SlidePower = Math.min(command/HighJunction, SlidePower_Up);
//
//            Aurelius.setPower(Aura_Robot.AuraMotors.CAT_MOUSE, SlidePower);
//            slide_currentPos = Aurelius.getCurrentPosition(Aura_Robot.AuraMotors.CAT_MOUSE);
//        }
//
//        telemetry.addData("rykUpSlide_pid: Current Slide Position: ", Aurelius.getCurrentPosition(Aura_Robot.AuraMotors.CAT_MOUSE));
//        telemetry.update();
//    }

//    public void MrvkTurret() {
//        if (Aurelius.getCurrentPosition(Aura_Robot.AuraMotors.CAT_MOUSE) > slideHeightSafetyBarrier || Aurelius.FlameThrower.getPosition() <= xSlideSafetyBarrier){
//            turret_Range[0] = turretRight;
//            turret_Range[1] = turretLeft;
//            telemetry.addData("Turret", "Full range is ready to go!");
//        }
//        else{
//            if (Claw_Position == Claw_Open_Pos){
//                turret_Range[0] = turret_restrictedRange[0];
//                turret_Range[1] = turret_restrictedRange[1];
//            }else{
//                turret_Range[0] = turretUp;
//                turret_Range[1] = turretUp;
//            }
//            telemetry.addData("Turret", "Full rotation is locked. Turn the turret forward/extend horizontal to unlock full range!");
//        }
//        telemetry.update();
//
//        //joystick control
//        turret_newPos += turretIncrement * -gamepad2.right_stick_x;
//        if (turret_newPos >= turret_Range[1]) {
//            turret_newPos = turret_Range[1];a
//        } else if (turret_newPos <= turret_Range[0]) {
//            turret_newPos = turret_Range[0];
//        }
//
//        //dPad control
//        if (gamepad2.dpad_left) {
//            if(turretLeft >= turret_Range[0] && turretLeft <= turret_Range[1]) {
//                turret_newPos = turretLeft;
//                telemetry.addLine("dPad left triggered. Set turret to left");
//                telemetry.update();
//            }
//        }
//
//        if (gamepad2.dpad_right) {
//            if(turretRight >= turret_Range[0] && turretRight <= turret_Range[1]) {
//                turret_newPos = turretRight;
//                telemetry.addLine("dPad right triggered. Set turret to right");
//                telemetry.update();
//            }
//        }
//
//        if (gamepad2.dpad_up) {
//            if(turretUp >= turret_Range[0] && turretUp <= turret_Range[1]) {
//                turret_newPos = turretUp;
//                telemetry.addLine("dPad up triggered. Set turret to forward");
//                telemetry.update();
//            }
//        }
//
//        if (gamepad2.dpad_down) {
//            if(turretDown >= turret_Range[0] && turretDown <= turret_Range[1]) {
//                turret_newPos = turretDown;
//                telemetry.addLine("dPad down triggered. Set turret to down");
//                telemetry.update();
//            }
//        }
//
//        //actually setting the position
//        if( turret_newPos != turret_currentPos  && turret_newPos >= turret_Range[0] && turret_newPos <= turret_Range[1] ) {
//            turret_Move = (turret_newPos - turret_currentPos) * turretSpeed;
//            Aurelius.Teacup.setPosition(turret_currentPos + turret_Move);
//            telemetry.addData("", turret_currentPos + turret_Move);
//            telemetry.update();
//            turret_currentPos = Aurelius.Teacup.getPosition();
//        }
    }
}