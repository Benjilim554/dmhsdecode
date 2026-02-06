package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.util.Range.clip;

import android.annotation.SuppressLint;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystem.util.PIDFController;

//We are so back
@Configurable
@TeleOp(name = "24152 TeleOp")
public class Dababy extends LinearOpMode {
    PIDFController flywheelController =
            new PIDFController(0, 0, 0.0, 0.0, 0, 0.0, 0.1338); // CHANGE PIDFVAS

    public static double targetVelocity, velocity;
    private double nominalVoltage = 12.5;

    private double desiredPower = 1;
    private double batteryVoltage, correctedPower;

    private double ctrlPow = 2.0;
    double error;
    double curVelocity;
    double curTargetVelocity = 1240;
    double farTargetVelocity = 2000;
    private static final double BASE_F = 14.5;
    private static final double BASE_P = 0.4;
    double F = 14.02;
    double P = 0.015;
    private double rx;
    private boolean doorToggle = false;

    //MOTORS
    private DcMotorEx leftShooterMotor, rightShooterMotor, intakeMotor;
    private DcMotorEx leftBackMotor, leftFrontMotor, rightBackMotor, rightFrontMotor;
    private CRServo transferServo, leftAxonHood, rightAxonHood;

    //LIMELIGHT
    private Limelight3A Limelight;
    private IMU imu;

    //LIMELIGHT TUNING VALUES
    private static final double kTurn = 0.02;      // how hard it corrects
    private static final double maxTurn = 0.25;    // clamp so it doesn’t yoink the wheel
    private static final double TICKS_PER_MOTOR_REV = 28.0;
    private static final double GEAR_RATIO_MOTOR_TO_FLYWHEEL = 1.0; // change later

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        leftShooterMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftHoodMotor"); // change directions if needed
        rightShooterMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightHoodMotor");
        intakeMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "intakeMotor"); // change directions if needed

        leftBackMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftBackMotor");
        leftFrontMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftFrontMotor");
        rightBackMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightBackMotor");
        rightFrontMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightFrontMotor");

        transferServo = (CRServo) hardwareMap.get(CRServo.class, "transferServo");
        leftAxonHood = (CRServo) hardwareMap.get(CRServo.class, "axonPowerLeft");
        rightAxonHood = (CRServo) hardwareMap.get(CRServo.class, "axonPowerRight");

        Limelight = hardwareMap.get(Limelight3A.class, "limelight");
        Limelight.pipelineSwitch(1); // pipeline 1 is all the tags, pipeline 2 is tag 22 and pipeline 3 is tag 24

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot hubOrientation =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                );
        imu.initialize(new IMU.Parameters(hubOrientation));

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();
        batteryVoltage = battery.getVoltage();
        leftShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE); //TODO: Change later bitch

        flywheelController.setFeedforward(
                0.00036, // kV
                0.0,    // kA (not needed for flywheel)
                0.065067     // kS
        );

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

                waitForStart();
                while (opModeIsActive()) {
                    telemetry.update();
                    previousGamepad1.copy(currentGamepad1);
                    currentGamepad1.copy(gamepad1);
                    batteryVoltage = battery.getVoltage();

                    //boilerplate, dont touch
                    rx = (gamepad1.right_stick_x / 4) * 3;

                    rightFrontMotor.setPower(Math.pow(gamepad1.left_stick_y + gamepad1.left_stick_x + rx, ctrlPow) * Math.signum(gamepad1.left_stick_y + gamepad1.left_stick_x + rx));
                    leftFrontMotor.setPower(Math.pow(gamepad1.left_stick_y - gamepad1.left_stick_x - rx, ctrlPow) * Math.signum(gamepad1.left_stick_y - gamepad1.left_stick_x - rx));
                    rightBackMotor.setPower(Math.pow(gamepad1.left_stick_y - gamepad1.left_stick_x + rx, ctrlPow) * Math.signum(gamepad1.left_stick_y - gamepad1.left_stick_x + rx));
                    leftBackMotor.setPower(Math.pow(gamepad1.left_stick_y + gamepad1.left_stick_x - rx, ctrlPow) * Math.signum(gamepad1.left_stick_y + gamepad1.left_stick_x - rx));


                    boolean lockOn = gamepad1.dpad_left; // okay so trigger doesnt work? so ima switch it to option, just cuz it would work
                    if (lockOn && tagDetected) {
                        double turnAssist = clip(kTurn * tx, -maxTurn, maxTurn);

                        double fwd = -gamepad2.left_stick_y;
                        double str = gamepad2.left_stick_x;
                        double trn = gamepad2.right_stick_x + turnAssist;

                        leftBackMotor.setPower(((fwd + str) - trn) * rx);
                        leftFrontMotor.setPower(((fwd - str) - trn) * rx);
                        rightBackMotor.setPower(((fwd - str) + trn) * rx);
                        rightFrontMotor.setPower(((fwd + str) + trn) * rx);

                        telemetry.addData("LockOn(OPTION)", "ON");
                        telemetry.addData("turnAssist", turnAssist);
                    } else {
                        telemetry.addData("LockOn(OPTIONS)", "OFF");
                    }

                    double currentVelocity = rightShooterMotor.getVelocity();
                    telemetry.addData("TargetVel", targetVelocity);
                    telemetry.addData("CurrentVel", velocity);
                    flywheelController.setPIDF(P, 0, 0.0, 0);
                    flywheelController.setFeedforward(0, 0, 0);
                    velocity = leftShooterMotor.getVelocity();

                    // MATA DRIVER 1: Drive, intake, transfer

                    if (gamepad1.right_trigger > 0.1) {
                        intakeMotor.setPower(0.5);
                    } else {
                        intakeMotor.setPower(0);
                    }

                    if (gamepad1.right_bumper) {
                        intakeMotor.setPower(-0.5);
                    } else {
                        intakeMotor.setPower(0);
                    }

                    if (gamepad1.square) {
                        transferServo.setPower(1);
                    } else {
                        transferServo.setPower(0);
                    }

                    if (gamepad1.circle) {
                        transferServo.setPower(-1);
                    } else {
                        transferServo.setPower(0);
                    }

                    // Transfer + Intake
                    if (gamepad1.triangle) {
                        transferServo.setPower(-.5);
                        intakeMotor.setPower(1);
                    } else {
                        transferServo.setPower(0);
                        intakeMotor.setPower(0);
                    }

                    // CHEVALIER DRIVER 2: flywheel, hood angle

                    if (gamepad2.right_trigger > 0.1) {
                        targetVelocity = 1240;
                        leftShooterMotor.setPower(1);
                        rightShooterMotor.setPower(1);
                        //leftShooterMotor.setPower(flywheelController.calculate(targetVelocity - velocity, targetVelocity, 0));
                        // rightShooterMotor.setPower(flywheelController.calculate(targetVelocity - velocity, targetVelocity, 0));

                    } else {
                        targetVelocity = 0;
                        leftShooterMotor.setPower(0.0);
                        rightShooterMotor.setPower(0.0);
                    }
                    if (gamepad2.right_bumper) {
                        targetVelocity = -1240;
                        leftShooterMotor.setPower(-1);
                        rightShooterMotor.setPower(-1);
                        //leftShooterMotor.setPower(flywheelController.calculate(targetVelocity - velocity, targetVelocity, 0));
                        //rightShooterMotor.setPower(flywheelController.calculate(targetVelocity - velocity, targetVelocity, 0));

                    } else {
                        targetVelocity = 0;
                        leftShooterMotor.setPower(0.0);
                        rightShooterMotor.setPower(0.0);
                    }

                    if (gamepad2.left_trigger > 0.1) {
                        leftAxonHood.setPower(-1);
                        rightAxonHood.setPower(-1);
                    } else {
                        leftAxonHood.setPower(0);
                        rightAxonHood.setPower(0);
                    }

                    if (gamepad2.left_bumper) {
                        leftAxonHood.setPower(1);
                        rightAxonHood.setPower(1);
                    } else {
                        leftAxonHood.setPower(0);
                        rightAxonHood.setPower(0);
                    }


                    curVelocity = rightShooterMotor.getVelocity();
                    error = curTargetVelocity + curVelocity;

                    telemetry.addData("Target velocity", curTargetVelocity);
                    telemetry.addData("Current Velocity", "%.2f", curVelocity);
                    telemetry.addData("Error", "%.2f", error);

                    telemetry.update();

                }
            }
        } finally {
            try { Limelight.stop(); } catch (Exception ignored) {}
        }
    }
}