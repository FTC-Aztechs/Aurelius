package org.firstinspires.ftc.teamcode;

public class Aura_AutoActions{
    //TODO: look at sample action class here and write your own
//    public class Shooter {
//        private DcMotorEx motor;
//
//        public Shooter(HardwareMap hardwareMap) {
//            motor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
//        }
//
//        public class SpinUp implements Action {
//            @Override
//            public void init() {
//                motor.setPower(0.8);
//            }
//
//            @Override
//            public boolean loop(TelemetryPacket packet) {
//                double vel = motor.getVelocity();
//
//                packet.put("shooterVelocity", vel);
//
//                return vel < 10_000.0;
//            }
//        }
//
//        public Action spinUp() {
//            return new SpinUp();
//        }
//    }
//
//    public class ShooterOpMode extends ActionOpMode {
//        @Override
//        public void runOpMode() throws InterruptedException {
//            Shooter shooter = new Shooter(hardwareMap);
//
//            waitForStart();
//
//            runBlocking(shooter.spinUp());
//        }
//    }
}
