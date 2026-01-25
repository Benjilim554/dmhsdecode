package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "Limelight SIMPLE Lock", group = "TeleOp")
public class THESTINK extends LinearOpMode {

    private DcMotor LFM;
    private DcMotor LBM;
    private DcMotor RFM;
    private DcMotor RBM;
    private DcMotor INTAKE;
    private CRServo servo;
    private DcMotorEx leftFlexWheel;
    private DcMotorEx rightFlexWheel;
    private DcMotorEx encoder;

    private Limelight3A Limelight;
    private IMU imu;


    // turn assist tuning
    private static final double kTurn = 0.02;      // how hard it corrects
    private static final double maxTurn = 0.25;    // clamp so it doesn’t yoink the wheel

    // RPM telemetry
    private static final double TICKS_PER_MOTOR_REV = 28.0;
    private static final double GEAR_RATIO_MOTOR_TO_FLYWHEEL = 1.0; // change later


    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        int SpeedFactor;
        int IntakeArmPos;
        boolean my_1PersonDrive;
        boolean holdingArmMotors;

        LFM = hardwareMap.get(DcMotor.class, "LFM");
        LBM = hardwareMap.get(DcMotor.class, "LBM");
        RFM = hardwareMap.get(DcMotor.class, "RFM");
        RBM = hardwareMap.get(DcMotor.class, "RBM");
        INTAKE = hardwareMap.get(DcMotor.class, "INTAKE");
        servo = hardwareMap.get(CRServo.class, "servo");
        encoder = hardwareMap.get(DcMotorEx.class, "encoder");
        leftFlexWheel = hardwareMap.get(DcMotorEx.class, "leftFlexWheel");
        rightFlexWheel = hardwareMap.get(DcMotorEx.class, "rightFlexWheel");

        leftFlexWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlexWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Limelight = hardwareMap.get(Limelight3A.class, "limelight");
        Limelight.pipelineSwitch(1); // pipeline 1 is all the tags, pipeline 2 is tag 22 and pipeline 3 is tag 24

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot hubOrientation =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                );
        imu.initialize(new IMU.Parameters(hubOrientation));

        my_1PersonDrive = false;
        SpeedFactor = 1;
        holdingArmMotors = false;

        LFM.setDirection(DcMotor.Direction.FORWARD);
        LBM.setDirection(DcMotor.Direction.FORWARD);
        RFM.setDirection(DcMotor.Direction.REVERSE);
        RBM.setDirection(DcMotor.Direction.FORWARD);

        IntakeArmPos = 0;

        telemetry.addLine("READY.");
        telemetry.addLine("Limelight telemetry should say TAG DETECTED if valid.");
        telemetry.update();

        waitForStart();

        Limelight.start();

        try {
            while (opModeIsActive()) {


                YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
                Limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));


                // This is the limelight part
                LLResult llResult = Limelight.getLatestResult();

                boolean tagDetected = (llResult != null && llResult.isValid());
                double tx = 0.0;
                double ta = 0.0;
                Pose3D botpose = null;

                if (tagDetected) {
                    tx = llResult.getTx();
                    ta = llResult.getTa();
                    botpose = llResult.getBotpose_MT2();
                }

                telemetry.addData("Limelight", tagDetected ? "TAG DETECTED" : "NOTHING IN SIGHT");
                telemetry.addData("Tag ID", "N/A");
                telemetry.addData("tx", tagDetected ? String.format("%.2f", tx) : "N/A");
                telemetry.addData("ta", tagDetected ? String.format("%.2f", ta) : "N/A");

                if (botpose != null) {
                    telemetry.addData("BotPose", botpose.toString());
                }


                //encoder stuff


                double leftTicksPerSec = leftFlexWheel.getVelocity();
                double rightTicksPerSec = rightFlexWheel.getVelocity();
                double flywheelVelocity = encoder.getVelocity();

                double leftMotorRpm  = (leftTicksPerSec  * 60.0) / TICKS_PER_MOTOR_REV;
                double rightMotorRpm = (rightTicksPerSec * 60.0) / TICKS_PER_MOTOR_REV;
                double encoderRpm = (flywheelVelocity * 60.0) / TICKS_PER_MOTOR_REV;

                double leftFlyRpm  = leftMotorRpm  / GEAR_RATIO_MOTOR_TO_FLYWHEEL;
                double rightFlyRpm = rightMotorRpm / GEAR_RATIO_MOTOR_TO_FLYWHEEL;
                double avgFlyRpm = (leftFlyRpm + rightFlyRpm) * 0.5;

                telemetry.addData("Encoder", String.format("%.1f", flywheelVelocity));
                telemetry.addData("Left RPM", String.format("%.1f", leftFlyRpm));
                telemetry.addData("Right RPM", String.format("%.1f", rightFlyRpm));
                telemetry.addData("Average RPM", String.format("%.1f", avgFlyRpm));

                //(DO NOT TOUCH)
                LBM.setPower(((-gamepad2.left_stick_y + gamepad2.left_stick_x) - gamepad2.right_stick_x) * SpeedFactor);
                LFM.setPower(((-gamepad2.left_stick_y - gamepad2.left_stick_x) - gamepad2.right_stick_x) * SpeedFactor);
                RBM.setPower(((-gamepad2.left_stick_y - gamepad2.left_stick_x) + gamepad2.right_stick_x) * SpeedFactor);
                RFM.setPower((-gamepad2.left_stick_y + gamepad2.left_stick_x + gamepad2.right_stick_x) * SpeedFactor);

                // the lock on limelight part
                boolean lockOn = gamepad1.options; // okay so trigger doesnt work? so ima switch it to option, just cuz it would work
                if (lockOn && tagDetected) {
                    double turnAssist = clip(kTurn * tx, -maxTurn, maxTurn);

                    double fwd = -gamepad2.left_stick_y;
                    double str = gamepad2.left_stick_x;
                    double trn = gamepad2.right_stick_x + turnAssist;

                    LBM.setPower(((fwd + str) - trn) * SpeedFactor);
                    LFM.setPower(((fwd - str) - trn) * SpeedFactor);
                    RBM.setPower(((fwd - str) + trn) * SpeedFactor);
                    RFM.setPower(((fwd + str) + trn) * SpeedFactor);

                    telemetry.addData("LockOn(OPTION)", "ON");
                    telemetry.addData("turnAssist", turnAssist);
                } else {
                    telemetry.addData("LockOn(OPTIONS)", "OFF");
                }

                // intake
                if (gamepad1.right_bumper) {
                    INTAKE.setPower(1);
                } else if (gamepad1.left_bumper) {
                    INTAKE.setPower(-1);
                } else if (gamepad1.y) {
                    INTAKE.setPower(1);
                } else {
                    INTAKE.setPower(0);
                }

                //brake
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

                // servo
                if (gamepad1.dpad_left) {
                    servo.setPower(0.85);
                } else if (gamepad1.dpad_right) {
                    servo.setPower(-0.85);
                } else {
                    servo.setPower(0);
                }
                if (gamepad1.dpad_up) {
                    SpinFlyWheel(-0.85, 1);
                } else if (gamepad1.a) {
                    SpinFlyWheel(1, -1);
                } else if (gamepad1.y) {
                    SpinFlyWheel(0.8, -1);
                } else if (gamepad1.dpad_down) {
                    SpinFlyWheel(-0.75, 1);
                } else if (gamepad1.x) {
                    SpinFlyWheel(0.9, -1);
                } else if (gamepad1.b) {
                    SpinFlyWheel(1, 1);
                } else {
                    SpinFlyWheel(0, 0);
                }

                telemetry.update();
            }
        } finally {
            try { Limelight.stop(); } catch (Exception ignored) {}
        }
    }

    private void SpinFlyWheel(double Speed, int Dirrection) {
        leftFlexWheel.setPower(Speed * Dirrection);
        rightFlexWheel.setPower(Speed * -Dirrection);

    }
    private double clip(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
    {

    }
}