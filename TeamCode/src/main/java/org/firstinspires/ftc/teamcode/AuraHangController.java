package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.AuraRobot.HangExtend;
import static org.firstinspires.ftc.teamcode.AuraRobot.HangIdle;
import static org.firstinspires.ftc.teamcode.AuraRobot.HangRotate;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class AuraHangController {
    public Servo hanger;

    public Servo funky;
    enum HangState
    {
        Idle,
        Hang,
        Up
    }
    AuraHangController.HangState currState;
    AuraHangController.HangState targetState;
    Telemetry telemetry;

    public AuraHangController(HardwareMap hardwareMap) {
        hanger = hardwareMap.get(Servo.class, "Hang");
        currState = AuraHangController.HangState.Idle;
    }

    public void setTelemetry(Telemetry tele)
    {
        telemetry = tele;
    }

    public void setTargetState(AuraHangController.HangState state) {
        targetState = state;
    }

    public void update() {
        //  Open -> Open: No-op
        //  Open -> Close: Close, update curr
        //  Open -> Auto: Set in Auto, update curr
        //  Close -> Open: Open, update curr
        //  Close->Close: No-op
        //  Close->Auto: set in Auto, update curr
        //  Auto->Open: Open, update curr
        //  Auto->Close: Close, update curr
        //  Auto->Auto : Auto, update curr
        if (currState == targetState)
            return;

        switch (targetState) {
            case Hang:
                hanger.setPosition(HangExtend);
                currState = AuraHangController.HangState.Idle;
                break;
            case Up:
                funky.setPosition(HangRotate);
                currState = HangState.Up;
            case Idle:
                hanger.setPosition(HangIdle);
                funky.setPosition(0);
                currState = AuraHangController.HangState.Hang;
                break;
        }

        return;
    }
}