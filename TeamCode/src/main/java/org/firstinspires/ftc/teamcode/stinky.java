package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name="Please work or I quit robotics", group="Test")
public class stinky extends OpMode {

    // drive motors
    private DcMotor LFM, RFM, LBM, RBM;

    // flywheel motors
    private DcMotorEx leftFlywheel, rightFlywheel;

    // imu and limelight
    private IMU imu;
    private Limelight3A limelight;

    // DRIVE TUNING FIX
    private double kStrafe = 0.02;   // tx -> strafe
    private double kForward = 0.06;  // (taTarget - ta) -> forward/back
    private double kTurn = 0.015;    // tx -> turn (align mode)

    // tag lock
    private double kTurnTrackNormal = 0.02; // aim assist
    private double kTurnTrackSniper = 0.01; // aim for sniping

    private double taTarget = 1.20;  // distance setpoint (TODO: calibrate on field)
    private double txTol = 1.0;      // deg
    private double taTol = 0.10;     // area percentage

    private double maxStrafe = 0.35;
    private double maxForward = 0.35;
    private double maxTurn = 0.25;

    // FLYWHEEL TUNING
    private static final double TICKS_PER_REV = 28.0; // TODO: fix for motor

    // basic P controller for RPM
    private double shooterKp = 0.0004;     // how aggressive the correction is
    private double shooterMaxPower = 1.0;  // don't go over this

    private double shooterTargetRpm = 0.0; // what RPM we want right now

    // steph curry mode
    // if ta is below this, it treat it as "far shot" zone
    private double sniperTaThreshold = 0.40; // TODO: tune this on field

    // normal-range RPM curve
    private double normalMinRpm = 2500; // close-ish
    private double normalMaxRpm = 3500; // mid

    // sniper-range RPM curve
    private double sniperMinRpm = 3400; // farther
    private double sniperMaxRpm = 4200; // max nuke

    // helpers(what the fluff does this mean)
    private double clip(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    private void driveRobotCentric(double forward, double strafe, double turn) {
        // mecanum mixer
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
        if (leftFlywheel == null || rightFlywheel == null) return 0;

        // velocity is ticks / second
        double leftVel = leftFlywheel.getVelocity();
        double rightVel = rightFlywheel.getVelocity();
        double avgVel = (leftVel + rightVel) * 0.5;

        // RPM
        return (avgVel / TICKS_PER_REV) * 60.0;
    }

    // "simple" P controller for shooter RPM
    private void updateShooter(double targetRpm) {
        shooterTargetRpm = targetRpm;

        if (leftFlywheel == null || rightFlywheel == null) return;

        if (targetRpm <= 0) {
            // chill
            leftFlywheel.setPower(0);
            rightFlywheel.setPower(0);
            return;
        }

        double currentRpm = getShooterRpm();
        double error = shooterTargetRpm - currentRpm;

        double power = shooterKp * error;

        // only spin forward; don't back-drive the flywheel
        power = clip(power, 0, shooterMaxPower);

        leftFlywheel.setPower(power);
        rightFlywheel.setPower(power);

        telemetry.addData("Shooter target RPM", shooterTargetRpm);
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
            minRpm = sniperMinRpm;
            maxRpm = sniperMaxRpm;
        } else {
            // NORMAL CURVE
            minRpm = normalMinRpm;
            maxRpm = normalMaxRpm;
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

        // shooter motors
        leftFlywheel  = hardwareMap.get(DcMotorEx.class, "leftFlexWheel");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "rightFlexWheel");

        leftFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot hubOrientation =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
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
        if (alignButton && hasTarget) {
            // FULL AUTO-ALIGN MODE (R2)
            double txErr = tx;               // want 0
            double taErr = (taTarget - ta);  // want 0

            double strafe  = clip(kStrafe * txErr, -maxStrafe,  maxStrafe);
            double forward = clip(kForward * taErr, -maxForward, maxForward);
            double turn    = clip(kTurn * txErr, -maxTurn,     maxTurn);

            driveRobotCentric(forward, strafe, turn);

            boolean aligned = Math.abs(txErr) <= txTol && Math.abs(taErr) <= taTol;

            telemetry.addData("MODE", aligned ? "ALIGN LOCKED (R2)" : "ALIGNING (R2)");
            telemetry.addData("tx", tx);
            telemetry.addData("ta", ta);
            telemetry.addData("yawDeg", yawDeg);
            telemetry.addData("txErr", txErr);
            telemetry.addData("taErr", taErr);
            telemetry.addData("RangeMode", sniperRange ? "SNIPER" : "NORMAL");
        } else {
            // DRIVER MODE
            // L2 = hold for aim assist (tag lock)
            double turnOut = turnCmd;

            if (hasTarget && tagLockButton) {
                double txErr = tx;

                // use softer turn when sniping so it's not twitchy
                double kTrack = sniperRange ? kTurnTrackSniper : kTurnTrackNormal;
                turnOut = clip(kTrack * txErr, -maxTurn, maxTurn);

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

        double targetRpm = 0.0;
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
    }

    @Override
    public void stop() {
        limelight.stop();
        driveRobotCentric(0,0,0);
        updateShooter(0.0);
    }
}