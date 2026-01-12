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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

@TeleOp(name="Please work or I quit robotics", group="Test")
public class stink extends OpMode {

    // ===== DRIVE MOTORS =====
    private DcMotor LFM, RFM, LBM, RBM;

    // ===== SHOOTER MOTORS (ENCODER) =====
    private DcMotorEx leftFlexWheel, rightFlexWheel;

    // ===== INTAKE + SERVO =====
    private DcMotor INTAKE;
    private CRServo servo;

    // ===== SENSORS =====
    private IMU imu;
    private Limelight3A limelight;

    // ===== TAG FILTER =====
    private static final int TAG_A = 22;
    private static final int TAG_B = 24;

    // ===== ENCODER / RPM SETTINGS =====
    // REV internal encoder is commonly 28 ticks per motor rev (on motor shaft)
    private static final double ENCODER_TICKS_PER_MOTOR_REV = 28.0;

    // If you have an external gearbox between motor and output shaft, set this.
    // If none, leave 1.0
    private static final double GEARBOX_RATIO = 1.0;

    // you said it's 5:3 (flywheel spins faster than output)
    // flywheel rpm = output rpm * (5/3)
    private static final double OUTPUT_TO_FLYWHEEL_RATIO = 5.0 / 3.0;

    // ===== DRIVER INPUT =====
    private static final double TRIGGER_ON = 0.80;

    // ===== LIMELIGHT AIM =====
    private final double sniperTaThreshold = 0.40; // ta small = far

    // Full align (R2)
    private final double kStrafe = 0.02;
    private final double kForward = 0.06;
    private final double kTurnAlign = 0.015;

    private final double maxStrafe = 0.35;
    private final double maxForward = 0.35;
    private final double maxTurnAlign = 0.25;

    private final double taTarget = 1.20; // tune later
    private final double txTol = 1.0;
    private final double taTol = 0.10;

    // Tag lock (L2) turn assist
    private final double kTurnTrackNormal = 0.02;
    private final double kTurnTrackSniper = 0.01;
    private final double maxTurnAssist = 0.25;

    // ===== SERVO SCALING BASED ON RPM =====
    private final double servoRpmStartSlow = 1500;
    private final double servoRpmFullSlow  = 3500;
    private final double servoMinScale     = 0.20;

    // =========================================================
    //                 PIDF TUNER (INTEGRATED)
    // Hold GAMEPAD1 Right Stick Button (R3) to tune live
    // =========================================================
    private boolean tunerHeld = false;

    private double tunerHighFlywheelRpm = 4200;
    private double tunerLowFlywheelRpm  = 3000;
    private double tunerTargetFlywheelRpm = tunerHighFlywheelRpm;

    private double P = 0.0;
    private double I = 0.0;
    private double D = 0.0;
    private double F = 0.0;

    private final double[] steps = new double[]{1.0, 0.1, 0.01, 0.001, 0.0001};
    private int stepIndex = 1; // start at 0.1

    // edge detect for tuner controls
    private boolean lastY=false, lastB=false, lastUp=false, lastDown=false, lastLeft=false, lastRight=false;

    // ===== HELPERS =====
    private double clip(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    // ===== "YOUR CAPTAIN LIKES IT" DRIVE CODE WRAPPER =====
    private void driveRobotCentric(double forward, double strafe, double turn) {
        double lf = forward + strafe + turn;
        double rf = forward - strafe - turn;
        double lb = forward - strafe + turn;
        double rb = forward - strafe - turn;

        double max = Math.max(1.0, Math.max(Math.abs(lf),
                Math.max(Math.abs(rf), Math.max(Math.abs(lb), Math.abs(rb)))));

        lf /= max; rf /= max; lb /= max; rb /= max;

        LFM.setPower(lf);
        RFM.setPower(rf);
        LBM.setPower(lb);
        RBM.setPower(rb);
    }

    // ===== RPM math (for telemetry + servo scaling) =====
    private double getMotorShaftRpmFromEncoders() {
        if (leftFlexWheel == null || rightFlexWheel == null) return 0.0;

        double leftVel = leftFlexWheel.getVelocity();   // ticks/sec
        double rightVel = rightFlexWheel.getVelocity(); // ticks/sec
        double avgVel = (leftVel + rightVel) * 0.5;

        return (avgVel / ENCODER_TICKS_PER_MOTOR_REV) * 60.0;
    }

    private double getOutputShaftRpm() {
        return getMotorShaftRpmFromEncoders() / GEARBOX_RATIO;
    }

    private double getFlywheelRpm() {
        return getOutputShaftRpm() * OUTPUT_TO_FLYWHEEL_RATIO;
    }

    // ===== servo scaling =====
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

    // ===== tag pick =====
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

    // ===== YOUR ORIGINAL HELPER (kept) =====
    private void SpinFlyWheel(double Speed, int Dirrection) {
        leftFlexWheel.setPower(Speed * Dirrection);
        rightFlexWheel.setPower(Speed * -Dirrection);
    }

    // ===== PIDF apply + setVelocity conversions =====
    private void applyPIDF() {
        PIDFCoefficients pidf = new PIDFCoefficients(P, I, D, F);
        leftFlexWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        rightFlexWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }

    // Flywheel RPM -> motor ticks/sec (for setVelocity)
    private double flywheelRpmToMotorTicksPerSec(double flywheelRpm) {
        // flywheel rpm = output rpm * OUTPUT_TO_FLYWHEEL_RATIO
        double outputRpm = flywheelRpm / OUTPUT_TO_FLYWHEEL_RATIO;

        // output rpm = motor rpm / GEARBOX_RATIO  => motor rpm = output rpm * GEARBOX_RATIO
        double motorRpm = outputRpm * GEARBOX_RATIO;

        // ticks/sec = motor rpm * ticks/rev / 60
        return (motorRpm * ENCODER_TICKS_PER_MOTOR_REV) / 60.0;
    }

    // motor ticks/sec -> flywheel rpm (for telemetry)
    private double motorTicksPerSecToFlywheelRpm(double ticksPerSec) {
        double motorRpm = (ticksPerSec * 60.0) / ENCODER_TICKS_PER_MOTOR_REV;
        double outputRpm = motorRpm / GEARBOX_RATIO;
        return outputRpm * OUTPUT_TO_FLYWHEEL_RATIO;
    }

    @Override
    public void init() {
        // drive motors
        LBM = hardwareMap.get(DcMotor.class, "LBM");
        LFM = hardwareMap.get(DcMotor.class, "LFM");
        RBM = hardwareMap.get(DcMotor.class, "RBM");
        RFM = hardwareMap.get(DcMotor.class, "RFM");

        // intake + servo
        INTAKE = hardwareMap.get(DcMotor.class, "INTAKE");
        servo = hardwareMap.get(CRServo.class, "servo");

        // shooter motors
        leftFlexWheel  = hardwareMap.get(DcMotorEx.class, "leftFlexWheel");
        rightFlexWheel = hardwareMap.get(DcMotorEx.class, "rightFlexWheel");
        leftFlexWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlexWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // your drive directions (kept)
        LFM.setDirection(DcMotor.Direction.FORWARD);
        LBM.setDirection(DcMotor.Direction.FORWARD);
        RFM.setDirection(DcMotor.Direction.REVERSE);
        RBM.setDirection(DcMotor.Direction.FORWARD);

        // imu
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

        // initial PIDF apply for tuner mode
        applyPIDF();

        telemetry.addLine("Init done.");
        telemetry.addLine("GAMEPAD2 drives (your original code).");
        telemetry.addLine("GP1: R2 full align | L2 tag-lock (turn only)");
        telemetry.addLine("Tracking ONLY tags: 22 + 24");
        telemetry.addLine("HOLD GP1 R3 (right stick button) = PF TUNER MODE");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {

        // ==========================
        // 1) LIMELIGHT READ + PICK TAG
        // ==========================
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double yawDeg = orientation.getYaw(AngleUnit.DEGREES);
        limelight.updateRobotOrientation(yawDeg);

        LLResult r = limelight.getLatestResult();
        TagPick tag = pickBestWantedTag(r);

        boolean hasWantedTag = (tag != null);
        double tx = hasWantedTag ? tag.txDeg : 0.0;
        double ta = hasWantedTag ? tag.ta : 0.0;
        boolean sniperRange = hasWantedTag && (ta <= sniperTaThreshold);

        // ==========================
        // 2) INPUTS (gp1 assist buttons)
        // ==========================
        boolean alignButton   = gamepad1.right_trigger > TRIGGER_ON; // R2
        boolean tagLockButton = gamepad1.left_trigger  > TRIGGER_ON; // L2

        // PF tuner: hold R3
        tunerHeld = gamepad1.right_stick_button;

        // ==========================
        // 3) DRIVE CONTROL (keep your style)
        // ==========================
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

            // your exact drive code (only curves if L2 + tag)
            LBM.setPower(((-gamepad2.left_stick_y + gamepad2.left_stick_x) - gamepad2.right_stick_x + turnAssist) * SpeedFactor);
            LFM.setPower(((-gamepad2.left_stick_y - gamepad2.left_stick_x) - gamepad2.right_stick_x + turnAssist) * SpeedFactor);
            RBM.setPower(((-gamepad2.left_stick_y - gamepad2.left_stick_x) + gamepad2.right_stick_x + turnAssist) * SpeedFactor);
            RFM.setPower(( -gamepad2.left_stick_y + gamepad2.left_stick_x + gamepad2.right_stick_x + turnAssist) * SpeedFactor);
        }

        // ==========================
        // 4) INTAKE (your original)
        // ==========================
        if (gamepad1.right_bumper) {
            INTAKE.setPower(1);
        } else if (gamepad1.left_bumper) {
            INTAKE.setPower(-1);
        } else if (gamepad1.y) {
            INTAKE.setPower(1);
        } else {
            INTAKE.setPower(0);
        }

        // brake/float (your original)
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

        // ==========================
        // 5) SERVO
        // ==========================
        double requestedServoPower = 0.0;
        if (!tunerHeld) {
            // normal mode: your original servo controls
            if (gamepad1.dpad_left) requestedServoPower = 1.0;
            else if (gamepad1.dpad_right) requestedServoPower = -1.0;
        } else {
            // tuner mode: dpad is used for PF tuning, so servo stays off
            requestedServoPower = 0.0;
        }

        double flywheelRpmNow = getFlywheelRpm();
        double scaledServoPower = scaleServoByRpm(requestedServoPower, flywheelRpmNow);
        servo.setPower(scaledServoPower);

        // ==========================
        // 6) FLYWHEEL CONTROL
        // ==========================
        if (!tunerHeld) {
            // NORMAL MODE: keep your manual mapping EXACT
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
        } else {
            // TUNER MODE (hold R3): setVelocity() + PIDF live tuning
            boolean y = gamepad1.y;
            boolean b = gamepad1.b;
            boolean up = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;
            boolean left = gamepad1.dpad_left;
            boolean right = gamepad1.dpad_right;

            double step = steps[stepIndex];

            if (y && !lastY) {
                tunerTargetFlywheelRpm =
                        (Math.abs(tunerTargetFlywheelRpm - tunerHighFlywheelRpm) < 1e-6)
                                ? tunerLowFlywheelRpm
                                : tunerHighFlywheelRpm;
            }
            if (b && !lastB) {
                stepIndex = (stepIndex + 1) % steps.length;
            }
            if (up && !lastUp) P += step;
            if (down && !lastDown) P -= step;

            if (right && !lastRight) F += step;
            if (left && !lastLeft)   F -= step;

            if (P < 0) P = 0;
            if (F < 0) F = 0;

            applyPIDF();

            double targetTicksPerSec = flywheelRpmToMotorTicksPerSec(tunerTargetFlywheelRpm);
            leftFlexWheel.setVelocity(targetTicksPerSec);
            rightFlexWheel.setVelocity(targetTicksPerSec);

            double currentTicksPerSec = (leftFlexWheel.getVelocity() + rightFlexWheel.getVelocity()) * 0.5;
            double currentFlywheelRpm = motorTicksPerSecToFlywheelRpm(currentTicksPerSec);
            double err = tunerTargetFlywheelRpm - currentFlywheelRpm;

            telemetry.addLine("=== PF TUNER MODE (HOLD R3) ===");
            telemetry.addData("TargetFlywheelRPM", tunerTargetFlywheelRpm);
            telemetry.addData("CurrentFlywheelRPM", currentFlywheelRpm);
            telemetry.addData("ErrorRPM", err);
            telemetry.addData("P", P);
            telemetry.addData("F", F);
            telemetry.addData("Step", steps[stepIndex]);
            telemetry.addData("TargetTicksPerSec", targetTicksPerSec);

            lastY=y; lastB=b;
            lastUp=up; lastDown=down; lastLeft=left; lastRight=right;
        }

        // ==========================
        // 7) TELEMETRY (always)
        // ==========================
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
        leftFlexWheel.setPower(0);
        rightFlexWheel.setPower(0);
        servo.setPower(0);
        INTAKE.setPower(0);
    }
}
