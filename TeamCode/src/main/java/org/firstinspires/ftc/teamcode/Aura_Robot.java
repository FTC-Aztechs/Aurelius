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

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Locale;

@Config
public class Aura_Robot
{
    enum AuraMotors
    {
        UPPER_LEFT,
        LOWER_LEFT,
        UPPER_RIGHT,
        LOWER_RIGHT,
        INTAKE,
        CAT_MOUSE,
        ALL_DRIVES,
        ALL_ATTACHMENTS,
        ALL
    }

    enum AuraServos
    {
        FLAMETHROWER,
        CARTOON,
        TEACUP
    }

    enum AutoState
    {
        PRELOAD,
        TOPCONE,
        TOPMIDCONE,
        MIDCONE,
        BOTTOMMIDCONE,
        BOTTOMCONE,
        PARK,
        IDLE
    }

    /* Public OpMode members. */
    public DcMotor upper_right = null;
    public DcMotor upper_left = null;
    public DcMotor lower_left = null;
    public DcMotor lower_right = null;

    public DcMotor intake = null;

//    public DcMotor Jerry = null;
//    public DcMotor Tom = null;



//    public Aura_LiftController TomAndJerrySlide;

    public WebcamName Khimera = null;
    OpenCvWebcam Sauron = null;

    // speeds/times
    public static double UpAdjust = 10;
    public static double speedAdjust = 7;
    public static double bumperSpeedAdjust = 10;
    public static double dPadSpeedAdjust = 7;

    public static double SlidePower_Up= 1;
    public static double SlidePower_Down = -0.01;
    public static double SlidePower_Zero = 0.0;
    public static double SlidePower = 0.5;
    public static int slideTicks_stepSize = 600;

    public static double turretSpeed = 0.5;
    public static int BUTTON_TRIGGER_TIMER_MS = 500;

    //auto cycles
    public static int Red_cyclesToRun = 4;
    public static int Blue_cyclesToRun = 4;

    // Vuforia Class Members
    public static OpenGLMatrix lastLocation   = null;
//    public static VuforiaLocalizer vuforia    = null;
//    public static VuforiaTrackables targets   = null ;
    public static WebcamName webcamName       = null;
    public static final String VUFORIA_KEY =
            "AZRnab7/////AAABmTUhzFGJLEyEnSXEYWthkjhGRzu8klNOmOa9WEHaryl9AZCo2bZwq/rtvx83YRIgV60/Jy/2aivoXaHNzwi7dEMGoaglSVmdmzPF/zOPyiz27dDJgLVvIROD8ww7lYzL8eweJ+5PqLAavvX3wgrahkOxxOCNeKG9Tl0LkbS5R11ATXL7LLWeUv5FP1aDNgMZvb8P/u96OdOvD6D40Nf01Xf+KnkF5EXwNQKk1r7qd/hiv9h80gvBXMFqMkVgUyogwEnlK2BfmeUhGVm/99BiwwW65LpKSaLVPpW/6xqz9SyPgZ/L/vshbWgSkTB/KoERiV8MsW79RPUuQS6NTOLY32I/kukmsis3MFst5LP/d3gx";


    //dimensions for vuforia recognition
    public static final float mmPerInch        = 25.4f;
    public static final float mmTargetHeight   = 6 * mmPerInch;          // the height of the center of the target image above the floor
    public static final float halfField        = 72 * mmPerInch;
    public static final float halfTile         = 12 * mmPerInch;
    public static final float oneAndHalfTile   = 36 * mmPerInch;
    
//    public static MvrkPose2d Red_Start = new MvrkPose2d(35.5, 63.5, -90);
//    public static MvrkPose2d Red_PushSignal = new MvrkPose2d(33.75,3, -90);
//    public static MvrkPose2d Red_CycleStart = new MvrkPose2d(33.75,12, -90); //  x = 35.5
//    public static MvrkPose2d Red_PreloadStart = new MvrkPose2d(35.75, 12, -90 );
//    public static MvrkPose2d Red_MidStart = new MvrkPose2d(35.75, 36, -90); // used for middle junction preload
//    public static MvrkPose2d Red_CycleEnd  = new MvrkPose2d(51.5,12,-90); // 52
//    public static MvrkPose2d Red_CycleEnd2 = new MvrkPose2d(50, 12, -90);
//    public static MvrkPose2d Red_Park_Pos1 = new MvrkPose2d(56,12, -90); //59
//    public static MvrkPose2d Red_Park_Pos2 = new MvrkPose2d(30.5,12, -90);
//    public static MvrkPose2d Red_Park_Pos3 = new MvrkPose2d(6.5,12, -90);
//
//    public static MvrkPose2d Blue_Start = new MvrkPose2d(-35.5, 63.5, -90);
//    public static MvrkPose2d Blue_PushSignal = new MvrkPose2d(-33.75,3, -90);
//    public static MvrkPose2d Blue_CycleStart = new MvrkPose2d(-33.75,12, -90); //  x = -35.5
//    public static MvrkPose2d Blue_PreloadStart = new MvrkPose2d(-35.75, 12, -90 );
//    public static MvrkPose2d Blue_MidStart = new MvrkPose2d(-35.75, 36, -90); // used for middle junction preload
//    public static MvrkPose2d Blue_CycleEnd  = new MvrkPose2d(-51.5,12,-90); // 52
//    public static MvrkPose2d Blue_CycleEnd2 = new MvrkPose2d(-50, 12, -90);
//    public static MvrkPose2d Blue_Park_Pos1 = new MvrkPose2d(-56,12, -90); //59
//    public static MvrkPose2d Blue_Park_Pos2 = new MvrkPose2d(-30.5,12, -90);
//    public static MvrkPose2d Blue_Park_Pos3 = new MvrkPose2d(-6.5,12, -90);

        public static double Preload_offset2 = -2.4;
        public static double Preload_offset3 = -1.4; //-2.0
        public static double Preload_offset4 = -1.0;
        public static double Preload_offset5 = -0.4;
        public static double Preload_offset6 = -0.3;
        public static double Preload_offset7 = -0.2;
        public static double Preload_offset8 = -0.2;
        public static double Preload_offset10 = -0.9;
        public static double Preload_offset11 = -0.7;

        public static double Preload_wait1 = 0.4;
        public static double Preload_wait6 = 0.2;
        public static double Preload_wait7 = 0.0;
        public static double Preload_wait8 = 0.2;

        public static double Cycle_offset2 = -1.2;
        public static double Cycle_offset3 = -0.8;
        public static double Cycle_offset4 = -0.4;
        public static double Cycle_offset5 = 0;
        public static double Cycle_offset6 = -0.1;
        public static double Cycle_offset7 = 0;
        public static double Cycle_offset8 = 0;
        public static double Cycle_offset9 = -1.2;
        public static double Cycle_offset10 = -1;
        public static double Cycle_offset11 = -0.4;
        public static double Cycle_offset12 = -0.3;
        public static double Cycle_offset13 = -0.2;
        public static double Cycle_offset14 = -0.2;
        public static double Cycle_offset15 = -0.2;
        public static double Cycle_offset16 = 0;
        public static double Cycle_offset17 = 0;

        public static double Cycle_wait2 = 0;
        public static double Cycle_wait1 = 0.4;
        public static double Cycle_wait3 = 0;
        public static double Cycle_wait4 = 0;
        public static double Cycle_wait5 = 0.2;
        public static double Cycle_wait6 = 0.2;
        public static double Cycle_wait7 = 0.2;
        public static double Cycle_wait8 = 0.4;
        public static double Cycle_wait9 = 0;
        public static double Cycle_wait10 = 0;
        public static double Cycle_wait11 = 0;
        public static double Cycle_wait12 = 0.2;
        public static double Cycle_wait13 = 0.2;
        public static double Cycle_wait14 = 0.2;
        public static double Cycle_wait15 = 0;
        public static double Cycle_wait16 = 0;
        public static double Cycle_wait17 = 0;

        public static double Dropoff_offset = -0.2;


//    public static Offsets PreloadOffsets;
//    public static Offsets CycleOffsets;
//    public static Waits PreloadWaits;
//    public static Waits CycleWaits;

    //claw variables
    public static double Claw_Open_Pos = 0.56;
    public static double Claw_Close_Pos = 0.75;

    //Tilt Positions
    public static double Tilted_Towers_Tilted_Pos = 0.8; //  Needs to change
    public static double Tilted_Towers_Straight_Pos = 0.41; // needs to change

    //Flamethrower variables
    public static double xSlideOutPos = 0.12;
    public static double xSlideDropPos = 0.25; //0.45; //0.5;
    public static double xSlidePickupPos = 0.17; //todo: eliminate?
    public static double xSlideInPos = 0.58;
    public static double xSlideInterPos = 0.49;

    public static double xSlideMaxExtension = xSlideOutPos;
    public static double xSlideMinExtension = xSlideInPos;

    public static double xSlideIncrement = 0.1;
        //minimum extension when the turret is past the restricted range, so it doesn't crash into anything
    public static double xSlideSafetyBarrier = 0.32;

    //Slide variables
    public static int LowerLimit     = 0;
    public static int FloorPosition  = 600;
    public static int BottomCone     = 740; //before 600 before 1240
    public static int GroundJunction = 1940;
    public static int BottomMidCone  = 1600; //1280; //2460
    public static int MiddleCone     = 2430; //2530; //2830
    public static int TopMidCone     = 3080; //3180; //3130
    public static int TopCone        = 3500;//3600; //3830; // 3730
    public static int LowJunction    = 6400; //5000; 6400
    public static int MidJunction    = 10850; // 10850
    public static int DropOffPos     = 13860;
    public static int MidDropOffPos     = 8300;
    public static int HighJunction   = 15400; // 15400
    public static int HighJunction_Auto = 15250;
    public static int UpperLimit     = 18000;

        //minimum height when the turret is past the restricted range, so it doesn't crash into anything
    public static int slideHeightSafetyBarrier = 5000;
    public static int slideHeightMinExtension = LowerLimit;
    public static int slideHeightMaxExtension = UpperLimit;

    //turret variables
    public static double[] turret_Range = {0.0, 0.0};
        //restricted range when slides/xSlide is not extended, to prevent the other stuff from crashing
    public static double[] turret_restrictedRange = {0.5, 0.59};
    public static double turretUp = 0.56;
    public static double turretDown = 0;
    public static double turretLeft = 0.84;
    public static double turretRight = 0.2675;
    public static double turretRedDropoff = 0.4138;
    public static double turretBlueDropoff = 0.6962;

    public static double turretIncrement = 0.005;
    public static double turretHalfRight = 0.40625;

//    public static MvrkPIDController slideUpPID = new MvrkPIDController(11, 0, 0, 5500); // KD Values .25 -> .32 KG Previous Values 3600 -> 5500 2/19/2023
//    public static MvrkPIDController slideDownPID = new MvrkPIDController(0.8, 0, 0, 1500);
//    public static double CycleExtendFlamethrowerOffset = -0.5;
//    public static double CycleRetractFlamethrowerOffset = -0.25;
//
//    public static Pose2d currentPose = new Pose2d();

    //device positions
    public static double Claw_Position; // Start at halfway position

    public static double xSlide_Position;

    public static double slide_newPos = FloorPosition;
    public static double slide_currentPos = 0;

    public static double turret_newPos = turretUp;
    public static double turret_Move;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();
    MecanumDrive AuraMecanumDrive;
    //public static MvrkPIDController control = new MvrkPIDController(11, 0, 0.25, 3600);

    /* Constructor */
    public Aura_Robot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        upper_right  = hwMap.get(DcMotor.class, "Upper_Right");
        upper_left = hwMap.get(DcMotor.class, "Upper_Left");
        lower_left = hwMap.get(DcMotor.class, "Lower_Left");
        lower_right = hwMap.get(DcMotor.class, "Lower_Right");

//        Jerry = hwMap.get(DcMotor.class, "Jerry");
//        Tom = hwMap.get(DcMotor.class, "Tom");


        //Servo
//        FlameThrower = hwMap.get(Servo.class, "Flamethrower");
//        Looney = hwMap.get(Servo.class, "Looney_Toons");
//        Teacup = hwMap.get(Servo.class, "Teacup");


        // Set all motors to zero power
        upper_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upper_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lower_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lower_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        Jerry.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        Tom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        upper_left.setDirection(DcMotor.Direction.REVERSE);  //-
        upper_right.setDirection(DcMotor.Direction.FORWARD); //+
        lower_left.setDirection(DcMotor.Direction.REVERSE); //- used to be
        lower_right.setDirection(DcMotor.Direction.FORWARD); //+ used to be

//        Jerry.setDirection(DcMotor.Direction.FORWARD); //- used to be
//        Tom.setDirection(DcMotor.Direction.FORWARD); //+ used to be

//        Looney.setDirection(Servo.Direction.FORWARD);
//        Teacup.setDirection(Servo.Direction.FORWARD);


        Pose2d initPose2d = new Pose2d(0,0,0);
        AuraMecanumDrive = new MecanumDrive(hwMap, initPose2d);
        Khimera = hwMap.get(WebcamName.class, "Kemera");

//        TomAndJerrySlide = new Mvrk_LiftController(hwMap);
//        TeacupTurret = new Mvrk_TurretController(hwMap);
//        LooneyClaw = new Mvrk_ClawController(hwMap);

    }
    String formatAngle( AngleUnit angleUnit, double angle) {
        return formatDegrees(angleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hwMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    public void setRunMode(AuraMotors eWhichMotor, DcMotor.RunMode eMode )
    {

        switch (eWhichMotor){
            case UPPER_LEFT:
                upper_left.setMode(eMode);
                break;
            case UPPER_RIGHT:
                upper_right.setMode(eMode);
                break;
            case LOWER_LEFT:
                lower_left.setMode(eMode);
                break;
            case LOWER_RIGHT:
                lower_right.setMode(eMode);
                break;
//            case CAT_MOUSE:
//                Jerry.setMode(eMode);
//                Tom.setMode(eMode);
//                break;
            case ALL_DRIVES:
                lower_right.setMode(eMode);
                lower_left.setMode(eMode);
                upper_right.setMode(eMode);
                upper_left.setMode(eMode);
                break;
            case ALL_ATTACHMENTS:
                break;
            case ALL:
//                lower_right.setMode(eMode);
//                lower_left.setMode(eMode);
//                upper_right.setMode(eMode);
//                upper_left.setMode(eMode);
//                Jerry.setMode(eMode);
//                Tom.setMode(eMode);
                break;
        }
    }

    public void setPower(AuraMotors eWhichMotor, double dPower )
    {

        switch (eWhichMotor){
            case UPPER_LEFT:
                upper_left.setPower(dPower);
                break;
            case UPPER_RIGHT:
                upper_right.setPower(dPower);
                break;
            case LOWER_LEFT:
                lower_left.setPower(dPower);
                break;
            case LOWER_RIGHT:
                lower_right.setPower(dPower);
                break;
//            case CAT_MOUSE:
//                Jerry.setPower(dPower);
//                Tom.setPower(dPower);
//                break;
            case ALL_DRIVES:
                lower_right.setPower(dPower);
                lower_left.setPower(dPower);
                upper_right.setPower(dPower);
                upper_left.setPower(dPower);
                break;
            case ALL:
//                lower_right.setPower(dPower);
//                lower_left.setPower(dPower);
//                upper_right.setPower(dPower);
//                upper_left.setPower(dPower);
//                Jerry.setPower(dPower);
//                Tom.setPower(dPower);
                break;
        }
    }

    public int getCurrentPosition( AuraMotors eWhichMotor )
    {
        switch(eWhichMotor)
        {
            case UPPER_LEFT:
                return upper_left.getCurrentPosition();
            case LOWER_LEFT:
                return lower_left.getCurrentPosition();
            case UPPER_RIGHT:
                return upper_right.getCurrentPosition();
            case LOWER_RIGHT:
                return lower_right.getCurrentPosition();
//            case CAT_MOUSE:
//                return Tom.getCurrentPosition();
            default:
                return 0;
        }
    }

    public void setPosition(AuraServos eWhichServo, double iPos )
    {
        switch( eWhichServo)
        {
//            case FLAMETHROWER:
//                FlameThrower.setPosition(iPos);
//                break;
//            case CARTOON:
//                Looney.setPosition(iPos);
//                break;
//            case TEACUP:
//                Teacup.setPosition(iPos);
//                break;
            default :
                break;
        }
    }



    public boolean areMotorsBusy(AuraMotors eWhichMotor) {

        switch(eWhichMotor)
        {
            case UPPER_LEFT: // upper left
                return upper_left.isBusy();
            case LOWER_LEFT: // lower left
                return lower_left.isBusy();
            case UPPER_RIGHT: // upper right
                return upper_right.isBusy();
            case LOWER_RIGHT: // lower right
                return lower_right.isBusy();
//            case CAT_MOUSE:
//                return Jerry.isBusy() && Tom.isBusy();
            case ALL_DRIVES: // All Drives
                return lower_left.isBusy() && lower_right.isBusy() && upper_left.isBusy() && upper_right.isBusy();
//            case ALL_ATTACHMENTS:
//                //return Linac.isBusy() && duck_wheel.isBusy() && Da_Winch.isBusy();
            case ALL:
//                return lower_left.isBusy() && lower_right.isBusy() && upper_left.isBusy() && upper_right.isBusy();
            default:
                return false;
        }
    }

    public void setTargetPosition(AuraMotors eWhichMotor, int iPos ) {
        switch (eWhichMotor) {
            case UPPER_LEFT:
                upper_left.setTargetPosition(iPos);
                break;
            case LOWER_LEFT:
                lower_left.setTargetPosition(iPos);
                break;
            case UPPER_RIGHT:
                upper_right.setTargetPosition(iPos);
                break;
            case LOWER_RIGHT:
                lower_right.setTargetPosition(iPos);
                break;
//            case CAT_MOUSE:
//                Jerry.setTargetPosition(iPos);
//                Tom.setTargetPosition(iPos);
//                break;
            case ALL_DRIVES:
                lower_right.setTargetPosition(iPos);
                lower_left.setTargetPosition(iPos);
                upper_right.setTargetPosition(iPos);
                upper_left.setTargetPosition(iPos);
                break;
            case ALL:
//                lower_right.setTargetPosition(iPos);
//                lower_left.setTargetPosition(iPos);
//                upper_right.setTargetPosition(iPos);
//                upper_left.setTargetPosition(iPos);
//                Jerry.setTargetPosition(iPos);
//                Tom.setTargetPosition(iPos);
            default:
                break;
        }
    }

}

