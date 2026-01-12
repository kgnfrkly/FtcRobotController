package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name="Please work or I quit robotics", group="Test")
public class stinker extends OpMode {

    // drive motors
    private DcMotor LFM, RFM, LBM, RBM;

    // flywheel motors
    private DcMotorEx leftFlexWheel, rightFlexWheel;

    private DcMotor INTAKE;

    private CRServo servo;

    // imu and limelight
    private IMU imu;
    private Limelight3A limelight;

    // FLYWHEEL TUNING
    private static final double TICKS_PER_REV = 28.0; // TODO: fix for motor

    // steph curry mode
    private final double sniperTaThreshold = 0.40; // TODO: tune this on field

    // helpers(what the fluff does this mean)
    private double clip(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    private void driveRobotCentric(double forward, double strafe, double turn) {
        // mixer
        double lf = forward + strafe + turn;
        double rf = forward - strafe - turn;
        double lb = forward - strafe + turn;
        double rb = forward + strafe - turn;

        // normalizer(what the fluff does this mean)
        double max = Math.max(1.0, Math.max(Math.abs(lf),
                Math.max(Math.abs(rf), Math.max(Math.abs(lb), Math.abs(rb)))));

        lf /= max; rf /= max; lb /= max; rb /= max;

        LFM.setPower(lf);
        RFM.setPower(rf);
        LBM.setPower(lb);
        RBM.setPower(rb);
    }

    // current RPM from encoders
    private double getShooterRpm() {
        if (leftFlexWheel == null || rightFlexWheel == null) return 0;

        // velocity is ticks / second
        double leftVel = leftFlexWheel.getVelocity();
        double rightVel = rightFlexWheel.getVelocity();
        double avgVel = (leftVel + rightVel) * 0.5;

        // RPM
        return (avgVel / TICKS_PER_REV) * 60.0;
    }

    // "simple" P controller for shooter RPM
    private void updateShooter(double targetRpm) {
        // what RPM we want right now

        if (leftFlexWheel == null || rightFlexWheel == null) return;

        if (targetRpm <= 0) {
            // chill
            leftFlexWheel.setPower(0);
            rightFlexWheel.setPower(0);
            return;
        }

        double currentRpm = getShooterRpm();
        double error = targetRpm - currentRpm;

        // basic P controller for RPM
        // how aggressive the correction is
        double shooterKp = 0.0004;
        double power = shooterKp * error;

        // only spin forward; don't back-drive the flywheel
        // don't go over this
        double shooterMaxPower = 1.0;
        power = clip(power, 0, shooterMaxPower);

        leftFlexWheel.setPower(power);
        rightFlexWheel.setPower(power);

        telemetry.addData("Shooter target RPM", targetRpm);
        telemetry.addData("Shooter current RPM", currentRpm);
        telemetry.addData("Shooter power", power);
    }

    // map Limelight info -> target shooter RPM (sniper-aware)
    private double getTargetRpmFromLimelight(double ta) {
        // ta small = far, ta big = close
        boolean sniper = ta <= sniperTaThreshold;

        // clamp ta so interpolation doesn't explode
        double taClamped = clip(ta, 0.15, 2.0);

        double minRpm, maxRpm;
        if (sniper) {
            // SNIPER CURVE
            // sniper-range RPM curve
            minRpm = 3400;
            maxRpm = 4200;
        } else {
            // NORMAL CURVE
            // normal-range RPM curve
            minRpm = 2500;
            maxRpm = 3500;
        }

        // distance factor based on ta
        double alpha = (2.0 - taClamped) / (2.0 - 0.15); // ~0..1
        double target = minRpm + alpha * (maxRpm - minRpm);

        telemetry.addData("SniperMode", sniper ? "ON" : "OFF");
        telemetry.addData("LL ta", ta);
        return target;

    }

    @Override
    public void init() {

        // driver motors
        LBM = hardwareMap.get(DcMotor.class, "LBM");
        LFM = hardwareMap.get(DcMotor.class, "LFM");
        RBM = hardwareMap.get(DcMotor.class, "RBM");
        RFM = hardwareMap.get(DcMotor.class, "RFM");

        INTAKE = hardwareMap.get(DcMotor.class, "INTAKE");
        servo = hardwareMap.get(CRServo.class, "servo");

        // shooter motors
        leftFlexWheel  = hardwareMap.get(DcMotorEx.class, "leftFlexWheel");
        rightFlexWheel = hardwareMap.get(DcMotorEx.class, "rightFlexWheel");

        leftFlexWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlexWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot hubOrientation =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                );
        imu.initialize(new IMU.Parameters(hubOrientation));

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(7); // your AprilTag pipeline index

        telemetry.addLine("Init done.");
        telemetry.addLine("R2 = full auto-align");
        telemetry.addLine("L2 = aim assist (tag lock) while held");
        telemetry.addLine("Shooter A = ON (LL RPM, sniper-aware)");
        telemetry.addLine("Shooter B = OFF");
        telemetry.update();

        LFM.setDirection(DcMotor.Direction.FORWARD);
        LBM.setDirection(DcMotor.Direction.FORWARD);
        RFM.setDirection(DcMotor.Direction.REVERSE);
        RBM.setDirection(DcMotor.Direction.FORWARD);

    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        // driver controls
        double strafeCmd  = gamepad1.left_stick_x;
        double forwardCmd = -gamepad1.left_stick_y;
        double turnCmd    = gamepad1.right_stick_x;

        // triggers on PS5 controller
        double alignTrigger  = gamepad1.right_trigger; // R2
        double tagLockTrigger = gamepad1.left_trigger; // L2

        // use a small threshold so slight accidental bumps don't activate
        boolean alignButton  = alignTrigger  > 0.5; // R2 for full auto-align
        boolean tagLockButton = tagLockTrigger > 0.5; // L2 for aim assist

        // update limelight with robot yaw (degrees)
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double yawDeg = orientation.getYaw(AngleUnit.DEGREES);
        limelight.updateRobotOrientation(yawDeg);

        LLResult r = limelight.getLatestResult();
        boolean hasTarget = (r != null && r.isValid());

        double tx = 0.0;
        double ta = 0.0;

        if (hasTarget) {
            tx = r.getTx();
            ta = r.getTa();
        }

        boolean sniperRange = hasTarget && (ta <= sniperTaThreshold);

        // DRIVE MODE
        double maxTurn = 0.25;
        if (alignButton && hasTarget) {
            // distance setpoint (TODO: calibrate on field)
            double taTarget = 1.20;
            double taErr = (taTarget - ta);  // want 0

            double kStrafe = 0.02;
            double maxStrafe = 0.35;
            double strafe  = clip(kStrafe * tx, -maxStrafe, maxStrafe);
            double kForward = 0.06;
            double maxForward = 0.35;
            double forward = clip(kForward * taErr, -maxForward, maxForward);
            double kTurn = 0.015;
            double turn    = clip(kTurn * tx, -maxTurn, maxTurn);
            driveRobotCentric(forward, strafe, turn);
            double txTol = 1.0;
            double taTol = 0.10;
            boolean aligned = Math.abs(tx) <= txTol && Math.abs(taErr) <= taTol;

            telemetry.addData("MODE", aligned ? "ALIGN LOCKED (R2)" : "ALIGNING (R2)");
            telemetry.addData("tx", tx);
            telemetry.addData("ta", ta);
            telemetry.addData("yawDeg", yawDeg);
            telemetry.addData("txErr", tx);
            telemetry.addData("taErr", taErr);
            telemetry.addData("RangeMode", sniperRange ? "SNIPER" : "NORMAL");
        } else {
            // DRIVER MODE
            // L2 = hold for aim assist (tag lock)
            double turnOut = turnCmd;

            if (hasTarget && tagLockButton) {
                // aim assist
                double kTurnTrackNormal = 0.02;
                // aim for sniping
                double kTurnTrackSniper = 0.01;
                double kTrack = sniperRange ? kTurnTrackSniper : kTurnTrackNormal;
                turnOut = clip(kTrack * tx, -maxTurn, maxTurn);

                telemetry.addData("MODE", "DRIVER + TAG LOCK (L2)");
                telemetry.addData("tx", tx);
                telemetry.addData("RangeMode", sniperRange ? "SNIPER" : "NORMAL");
            } else {
                // manual turn
                telemetry.addData("MODE", "DRIVER");
                telemetry.addData("RangeMode", hasTarget ? (sniperRange ? "SNIPER" : "NORMAL") : "NONE");
            }

            driveRobotCentric(forwardCmd, strafeCmd, turnOut);
            telemetry.addData("yawDeg", yawDeg);
            telemetry.addData("LL", hasTarget ? "target" : "none");
        }

        double targetRpm;
        if (gamepad1.a) {
            if (hasTarget) {
                targetRpm = getTargetRpmFromLimelight(ta);
            } else {
                // fallback if no tag but you still want shooter spun
                targetRpm = 3000; // TODO: tune this
            }
        } else if (gamepad1.b) {
            targetRpm = 0.0;
        } else {
            targetRpm = 0.0;
        }

        updateShooter(targetRpm);

        telemetry.update();

        float SpeedFactor = 1;
        LBM.setPower(((-gamepad2.left_stick_y + gamepad2.left_stick_x) - gamepad2.right_stick_x) * SpeedFactor);
            LFM.setPower(((-gamepad2.left_stick_y - gamepad2.left_stick_x) - gamepad2.right_stick_x) * SpeedFactor);
            RBM.setPower(((-gamepad2.left_stick_y - gamepad2.left_stick_x) + gamepad2.right_stick_x) * SpeedFactor);
            RFM.setPower((-gamepad2.left_stick_y + gamepad2.left_stick_x + gamepad2.right_stick_x) * SpeedFactor);
            
            if (gamepad1.right_bumper) {
                INTAKE.setPower(1);
            } else if (gamepad1.left_bumper) {
                INTAKE.setPower(-1);
            } else if (gamepad1.y) {
                INTAKE.setPower(1);
            } else {
                INTAKE.setPower(0);
            }
            if (gamepad2.x) {
                LBM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                LFM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                RBM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                RFM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else {
                LBM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                LFM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                RBM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                RFM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
            if (gamepad1.dpad_left) {
                servo.setPower(1);
            } else if (gamepad1.dpad_right) {
                servo.setPower(-1);
            } else {
                servo.setPower(0);
            }
            if (gamepad1.dpad_up) {
                SpinFlyWheel(0.5, 1);
            } else if (gamepad1.b) {
                SpinFlyWheel(1, 1);
            } else if (gamepad1.touchpad) {
                SpinFlyWheel(0.9, 1);
            } else if (gamepad1.dpad_down) {
                SpinFlyWheel(0.85, 1);
            } else if (false) {
                SpinFlyWheel(0.61, 1);
            } else if (gamepad1.a) {
                SpinFlyWheel(1, -1);
            } else {
                SpinFlyWheel(0, 0);
            }
        }


private void SpinFlyWheel(double Speed, int Dirrection) {
    leftFlexWheel.setPower(Speed * Dirrection);
    rightFlexWheel.setPower(Speed * -Dirrection);
}


    @Override
   public void stop() {
        limelight.stop();
        driveRobotCentric(0,0,0);
        updateShooter(0.0);
    }
}