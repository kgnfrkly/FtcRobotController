package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
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

import java.util.List;

@TeleOp(name="Please work or I quit robotics", group="Test")
public class stinkiest extends OpMode {

    private DcMotor LFM, RFM, LBM, RBM;

    private DcMotorEx leftFlexWheel, rightFlexWheel;

    private DcMotor INTAKE;
    private CRServo servo;

    private IMU imu;
    private Limelight3A limelight;

    private static final int TAG_A = 22;
    private static final int TAG_B = 24;

    private static final double ENCODER_TICKS_PER_MOTOR_REV = 28.0;

    // If you're using an external gearbox, set this (example: 19.2, 13.7, etc). If no gearbox, leave 1.0
    private static final double GEARBOX_RATIO = 1.0;
    private static final double OUTPUT_TO_FLYWHEEL_RATIO = 5.0 / 3.0;

    private static final double TRIGGER_ON = 0.80;

    private final double sniperTaThreshold = 0.40;

    private final double kStrafe = 0.02;
    private final double kForward = 0.06;
    private final double kTurnAlign = 0.015;

    private final double maxStrafe = 0.35;
    private final double maxForward = 0.35;
    private final double maxTurnAlign = 0.25;

    private final double taTarget = 1.20;

    private final double txTol = 1.0;
    private final double taTol = 0.10;

    private final double kTurnTrackNormal = 0.02;
    private final double kTurnTrackSniper = 0.01;
    private final double maxTurnAssist = 0.25;

    private final double shooterKp = 0.0004;
    private final double shooterMaxPower = 1.0;

    private final double servoRpmStartSlow = 1500;
    private final double servoRpmFullSlow  = 3500;
    private final double servoMinScale     = 0.20;

    private double clip(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    private void driveRobotCentric(double forward, double strafe, double turn) {
        double lf = forward + strafe + turn;
        double rf = forward - strafe - turn;
        double lb = forward - strafe + turn;
        double rb = forward + strafe - turn;

        double max = Math.max(1.0, Math.max(Math.abs(lf),
                Math.max(Math.abs(rf), Math.max(Math.abs(lb), Math.abs(rb)))));

        lf /= max; rf /= max; lb /= max; rb /= max;

        LFM.setPower(lf);
        RFM.setPower(rf);
        LBM.setPower(lb);
        RBM.setPower(rb);
    }

    private double getMotorShaftRpmFromEncoders() {
        if (leftFlexWheel == null || rightFlexWheel == null) return 0.0;

        double leftVel = leftFlexWheel.getVelocity();
        double rightVel = rightFlexWheel.getVelocity();
        double avgVel = (leftVel + rightVel) * 0.5;

        return (avgVel / ENCODER_TICKS_PER_MOTOR_REV) * 60.0;
    }

    private double getOutputShaftRpm() {
        return getMotorShaftRpmFromEncoders() / GEARBOX_RATIO;
    }

    private double getFlywheelRpm() {
        return getOutputShaftRpm() * OUTPUT_TO_FLYWHEEL_RATIO;
    }

    private void setShooterPower(double power) {
        power = clip(power, -1.0, 1.0);
        leftFlexWheel.setPower(power);
        rightFlexWheel.setPower(power);
    }

    private void updateShooterRpm(double targetFlywheelRpm) {
        if (targetFlywheelRpm <= 0) {
            setShooterPower(0);
            return;
        }

        double currentFlywheelRpm = getFlywheelRpm();
        double error = targetFlywheelRpm - currentFlywheelRpm;

        double power = shooterKp * error;
        power = clip(power, 0.0, shooterMaxPower);

        leftFlexWheel.setPower(power);
        rightFlexWheel.setPower(power);

        telemetry.addData("ShooterTargetRPM(FW)", targetFlywheelRpm);
        telemetry.addData("ShooterRPM(FW)", currentFlywheelRpm);
        telemetry.addData("ShooterPower", power);
    }

    private double getTargetRpmFromLimelight(double ta) {
        boolean sniper = ta <= sniperTaThreshold;

        double taClamped = clip(ta, 0.15, 2.0);

        double minRpm, maxRpm;
        if (sniper) {
            minRpm = 3400;
            maxRpm = 4200;
        } else {
            minRpm = 2500;
            maxRpm = 3500;
        }

        double alpha = (2.0 - taClamped) / (2.0 - 0.15);
        double target = minRpm + alpha * (maxRpm - minRpm);

        telemetry.addData("SniperMode", sniper ? "ON" : "OFF");
        telemetry.addData("LL ta (selected)", ta);
        return target;
    }

    private double scaleServoByRpm(double requestedPower, double flywheelRpm) {
        double scale;
        if (flywheelRpm <= servoRpmStartSlow) {
            scale = 1.0;
        } else if (flywheelRpm >= servoRpmFullSlow) {
            scale = servoMinScale;
        } else {
            double t = (flywheelRpm - servoRpmStartSlow) / (servoRpmFullSlow - servoRpmStartSlow);
            scale = 1.0 + t * (servoMinScale - 1.0);
        }
        return requestedPower * scale;
    }

    private static class TagPick {
        int id;
        double txDeg;
        double ta;
        TagPick(int id, double txDeg, double ta) {
            this.id = id;
            this.txDeg = txDeg;
            this.ta = ta;
        }
    }

    private TagPick pickBestWantedTag(LLResult r) {
        if (r == null || !r.isValid()) return null;

        List<LLResultTypes.FiducialResult> tags = r.getFiducialResults();
        if (tags == null || tags.isEmpty()) return null;

        TagPick best = null;

        for (LLResultTypes.FiducialResult fr : tags) {
            int id = fr.getFiducialId();
            if (id != TAG_A && id != TAG_B) continue;

            double tx = fr.getTargetXDegrees();
            double ta = fr.getTargetArea();

            if (best == null || ta > best.ta) best = new TagPick(id, tx, ta);
        }

        return best;
    }

    private void SpinFlyWheel(double Speed, int Dirrection) {
        leftFlexWheel.setPower(Speed * Dirrection);
        rightFlexWheel.setPower(Speed * -Dirrection);
    }

    @Override
    public void init() {

        LBM = hardwareMap.get(DcMotor.class, "LBM");
        LFM = hardwareMap.get(DcMotor.class, "LFM");
        RBM = hardwareMap.get(DcMotor.class, "RBM");
        RFM = hardwareMap.get(DcMotor.class, "RFM");


        INTAKE = hardwareMap.get(DcMotor.class, "INTAKE");
        servo = hardwareMap.get(CRServo.class, "servo");


        leftFlexWheel  = hardwareMap.get(DcMotorEx.class, "leftFlexWheel");
        rightFlexWheel = hardwareMap.get(DcMotorEx.class, "rightFlexWheel");
        leftFlexWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlexWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LFM.setDirection(DcMotor.Direction.FORWARD);
        LBM.setDirection(DcMotor.Direction.FORWARD);
        RFM.setDirection(DcMotor.Direction.REVERSE);
        RBM.setDirection(DcMotor.Direction.FORWARD);


        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot hubOrientation =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                );
        imu.initialize(new IMU.Parameters(hubOrientation));

        // limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(7);

        telemetry.addLine("Init done.");
        telemetry.addLine("GAMEPAD2 drives (your original code).");
        telemetry.addLine("GP1: R2 full align | L2 tag-lock (turn only)");
        telemetry.addLine("GP1: A auto RPM | B off");
        telemetry.addLine("Tracking ONLY tags: 22 + 24");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {


        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double yawDeg = orientation.getYaw(AngleUnit.DEGREES);
        limelight.updateRobotOrientation(yawDeg);

        LLResult r = limelight.getLatestResult();
        TagPick tag = pickBestWantedTag(r);

        boolean hasWantedTag = (tag != null);
        double tx = hasWantedTag ? tag.txDeg : 0.0;
        double ta = hasWantedTag ? tag.ta : 0.0;
        boolean sniperRange = hasWantedTag && (ta <= sniperTaThreshold);


        boolean alignButton   = gamepad1.right_trigger > TRIGGER_ON; // R2
        boolean tagLockButton = gamepad1.left_trigger  > TRIGGER_ON; // L2

        // IMPORTANT  ONLY write drive motor power ONCE per loop
        // R2/L2 override. Otherwise, chud drive code runs

        boolean usingAutoDrive = false;

        if (alignButton) {
            usingAutoDrive = true;

            if (hasWantedTag) {
                double taErr = (taTarget - ta);

                double strafe  = clip(kStrafe * tx,     -maxStrafe,    maxStrafe);
                double forward = clip(kForward * taErr, -maxForward,   maxForward);
                double turn    = clip(kTurnAlign * tx,  -maxTurnAlign, maxTurnAlign);

                driveRobotCentric(forward, strafe, turn);

                boolean aligned = Math.abs(tx) <= txTol && Math.abs(taErr) <= taTol;
                telemetry.addData("MODE", aligned ? "ALIGN LOCKED (R2)" : "ALIGNING (R2)");
            } else {
                // no tag -> stop so it doesn't wander
                driveRobotCentric(0, 0, 0);
                telemetry.addData("MODE", "ALIGN (R2) - NO TAG 22/24");
            }
        }

        if (!usingAutoDrive) {

            float SpeedFactor = 1;

            double turnAssist = 0.0;
            if (tagLockButton && hasWantedTag) {
                double kTrack = sniperRange ? kTurnTrackSniper : kTurnTrackNormal;
                turnAssist = clip(kTrack * tx, -maxTurnAssist, maxTurnAssist);
                telemetry.addData("MODE", "DRIVER + TAG LOCK (L2)");
            } else {
                telemetry.addData("MODE", "DRIVER");
            }

            // Your original drive code, but we add ONLY turnAssist to the turning term
            // (so it only curves when L2 held and tag exists)
            LBM.setPower(((-gamepad2.left_stick_y + gamepad2.left_stick_x) - gamepad2.right_stick_x + turnAssist) * SpeedFactor);
            LFM.setPower(((-gamepad2.left_stick_y - gamepad2.left_stick_x) - gamepad2.right_stick_x + turnAssist) * SpeedFactor);
            RBM.setPower(((-gamepad2.left_stick_y - gamepad2.left_stick_x) + gamepad2.right_stick_x + turnAssist) * SpeedFactor);
            RFM.setPower(( -gamepad2.left_stick_y + gamepad2.left_stick_x + gamepad2.right_stick_x + turnAssist) * SpeedFactor);
        }

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


        double requestedServoPower = 0.0;
        if (gamepad1.dpad_left) {
            requestedServoPower = 1.0;
        } else if (gamepad1.dpad_right) {
            requestedServoPower = -1.0;
        } else {
            requestedServoPower = 0.0;
        }

        double flywheelRpmNow = getFlywheelRpm();
        double scaledServoPower = scaleServoByRpm(requestedServoPower, flywheelRpmNow);
        servo.setPower(scaledServoPower);


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

        //  if want auto RPM on A instead of SpinFlyWheel, uncomment block and remove the A mapping above
        /*
        if (gamepad1.a) {
            double targetRpm = hasWantedTag ? getTargetRpmFromLimelight(ta) : 3000;
            updateShooterRpm(targetRpm);
        } else if (gamepad1.b) {
            updateShooterRpm(0);
        } else {
            updateShooterRpm(0);
        }
        */

        telemetry.addData("LL Valid", (r != null && r.isValid()));
        telemetry.addData("WantedTagSeen", hasWantedTag);
        telemetry.addData("TagID", hasWantedTag ? tag.id : -1);
        telemetry.addData("tx(deg)", tx);
        telemetry.addData("ta(%)", ta);
        telemetry.addData("RangeMode", hasWantedTag ? (sniperRange ? "SNIPER" : "NORMAL") : "NONE");
        telemetry.addData("YawDeg", yawDeg);

        telemetry.addData("MotorShaftRPM(enc)", getMotorShaftRpmFromEncoders());
        telemetry.addData("OutputShaftRPM", getOutputShaftRpm());
        telemetry.addData("FlywheelRPM", flywheelRpmNow);
        telemetry.addData("Output->FlywheelRatio", OUTPUT_TO_FLYWHEEL_RATIO);

        telemetry.addData("ServoRequested", requestedServoPower);
        telemetry.addData("ServoScaled", scaledServoPower);

        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
        driveRobotCentric(0, 0, 0);
        setShooterPower(0);
        servo.setPower(0);
        INTAKE.setPower(0);
    }
}
