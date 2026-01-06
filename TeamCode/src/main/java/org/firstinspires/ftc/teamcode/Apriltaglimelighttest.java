package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name="Mecanum + Limelight Align (1 file)", group="Test")
public class Apriltaglimelighttest extends OpMode {

    // sigma motors
    private DcMotor LFM, RFM, LBM, RBM;

    // sensors
    private IMU imu;
    private Limelight3A limelight;

    // tuning knobs
    private double kStrafe = 0.02;   // tx -> strafe
    private double kForward = 0.06;  // (taTarget - ta) -> forward/back
    private double kTurn = 0.015;    // tx -> turn (which is optional but who cares lolz)

    private double taTarget = 1.20;  // setpoint for distance (NOTE FOR FUTURE COLEMAN:(calibrate on field))
    private double txTol = 1.0;      // deg
    private double taTol = 0.10;     // area %

    private double maxStrafe = 0.35;
    private double maxForward = 0.35;
    private double maxTurn = 0.25;

    // helpers 
    private double clip(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    private void driveRobotCentric(double forward, double strafe, double turn) {
        // mecanum mixer thing
        double lf = forward + strafe + turn;
        double rf = forward - strafe - turn;
        double lb = forward - strafe + turn;
        double rb = forward + strafe - turn;

        // the sigma normalizer
        double max = Math.max(1.0, Math.max(Math.abs(lf),
                Math.max(Math.abs(rf), Math.max(Math.abs(lb), Math.abs(rb)))));

        lf /= max; rf /= max; lb /= max; rb /= max;

        LFM.setPower(lf);
        RFM.setPower(rf);
        LBM.setPower(lb);
        RBM.setPower(rb);
    }

    @Override
    public void init() {
        // boring config
        LBM = hardwareMap.get(DcMotor.class, "LBM");
        LFM = hardwareMap.get(DcMotor.class, "LFM");
        RBM = hardwareMap.get(DcMotor.class, "RBM");
        RFM = hardwareMap.get(DcMotor.class, "RFM");

        // TEST FOR FUTURE COLEMAN (if  robot drives backwards, flip these or reverse motors in config)
        // LFM.setDirection(DcMotor.Direction.REVERSE);
        // LBM.setDirection(DcMotor.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot hubOrientation =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                );
        imu.initialize(new IMU.Parameters(hubOrientation));


        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(7); // your AprilTag pipeline index

        telemetry.addLine("Init done. Hold RIGHT BUMPER to auto-align.");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        // driver controls
        double strafeCmd = gamepad1.left_stick_x;
        double forwardCmd = -gamepad1.left_stick_y;
        double turnCmd = gamepad1.right_stick_x;

        boolean alignButton = gamepad1.right_bumper; // hold to align

        // TEST FOR FUTURE COLEMAN(update limelight with robot yaw (degrees))
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double yawDeg = orientation.getYaw(AngleUnit.DEGREES);
        limelight.updateRobotOrientation(yawDeg);

        LLResult r = limelight.getLatestResult();
        boolean hasTarget = (r != null && r.isValid());

        if (alignButton) {
            if (hasTarget) {
                double tx = r.getTx();
                double ta = r.getTa();

                // errors
                double txErr = tx;                 // want 0
                double taErr = (taTarget - ta);    // want 0

                // p-control outputs
                double strafe = clip(kStrafe * txErr, -maxStrafe, maxStrafe);
                double forward = clip(kForward * taErr, -maxForward, maxForward);
                double turn = clip(kTurn * txErr, -maxTurn, maxTurn);

                driveRobotCentric(forward, strafe, turn);

                boolean aligned = Math.abs(txErr) <= txTol && Math.abs(taErr) <= taTol;

                telemetry.addData("MODE", aligned ? "ALIGN LOCKED" : "ALIGNING");
                telemetry.addData("tx", tx);
                telemetry.addData("ta", ta);
                telemetry.addData("yawDeg", yawDeg);
                telemetry.addData("txErr", txErr);
                telemetry.addData("taErr", taErr);
            } else {
                // no target =  stop drop roll and freeze so it doesnt wander like an idiot
                driveRobotCentric(0, 0, 0);
                telemetry.addData("MODE", "ALIGN (NO TARGET)");
                telemetry.addData("yawDeg", yawDeg);
            }
        } else {
            // normal driver mode
            driveRobotCentric(forwardCmd, strafeCmd, turnCmd);
            telemetry.addData("MODE", "DRIVER");
            telemetry.addData("yawDeg", yawDeg);
            telemetry.addData("LL", hasTarget ? "target" : "none");
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
        driveRobotCentric(0,0,0);
    }
}
