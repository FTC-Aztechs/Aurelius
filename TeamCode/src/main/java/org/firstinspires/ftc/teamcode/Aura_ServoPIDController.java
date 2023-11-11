package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Aura_ServoPIDController {
    public Servo controlledServo;
    double tgtPos;
    double currPos;
    public static double kP = 0.25;
    public static Aura_PIDController servoPID = new Aura_PIDController(kP, 0, 0 , 0);
    private Telemetry telemetry;

    public void setTelemetry(Telemetry tele) {
        telemetry = tele;
    }

    public Aura_ServoPIDController(HardwareMap hardwareMap, String servoName, double startPos, double endPos) {
        controlledServo = hardwareMap.get(Servo.class, servoName);
        tgtPos = startPos;
        currPos = startPos;
    }

    public void setTargetPosition(double target) {
        tgtPos = target;
    }

    public boolean update() {

        if(currPos != tgtPos ) {
            double posDelta = servoPID.output(tgtPos, currPos);
            controlledServo.setPosition(currPos+posDelta);
            currPos = controlledServo.getPosition();
            return false;
        }
        else
            return true;
    }

}