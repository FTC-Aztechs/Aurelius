package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

// Orthogonal #3; Logo RIGHT, USB UP

public class AuraHeadingEstimator {
    private IMU.Parameters myIMUParameters;
    private Telemetry telemetry;
    private IMU myIMU;

    public AuraHeadingEstimator(HardwareMap hwMap) {
        myIMU = hwMap.get(IMU.class, "imu");
        myIMUParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP)
        );
        myIMU.initialize(myIMUParameters);
    }

    public void resetYaw()
    {
        myIMU.resetYaw();
    }

    public void setTelemetry(Telemetry tele) {
        telemetry = tele;
    }

    public double getYaw() {
        // Yaw value should INCREASE as the robot is rotated Counter Clockwise. (Rotation about Z) <br>
        YawPitchRollAngles angles = myIMU.getRobotYawPitchRollAngles();
        return angles.getYaw(AngleUnit.RADIANS);
    }
}
