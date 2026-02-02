package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.subsystem.util.PIDFController;

//We are so back
@Configurable
@TeleOp(name = "7959 Teleop")
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

        transferServo = (CRServo) hardwareMap.get(DcMotor.class, "transferServo");
        leftAxonHood = (CRServo) hardwareMap.get(CRServo.class, "leftAxonHood");
        rightAxonHood = (CRServo) hardwareMap.get(CRServo.class, "rightAxonHood");


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();
        batteryVoltage = battery.getVoltage();
//        PIDFCoefficients pidfCoefficients1 = new PIDFCoefficients(P, 0, 0, F);
//        shooterMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients1);
//        shooterMotor3.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients1);
        leftShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //shooterMotor2 = leftShooterMotor
        rightShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //shooterMotor3 = rightShooterMotor

        flywheelController.setFeedforward(
                0.00036, // kV
                0.0,    // kA (not needed for flywheel)
                0.065067     // kS
        );
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





            double currentVelocity = rightShooterMotor.getVelocity();
            double kV = 0.00036;
            double I = 0;
            double kS = 0.065067;
            telemetry.addData("TargetVel", targetVelocity);
            telemetry.addData("CurrentVel", velocity);
            flywheelController.setPIDF(P, I, 0.0, 0);
            flywheelController.setFeedforward(kV, 0, kS);
            velocity = leftShooterMotor.getVelocity();


            if (gamepad2.right_trigger > 0.1) {
                targetVelocity = 1240;
                leftShooterMotor.setPower(flywheelController.calculate(targetVelocity - velocity, targetVelocity, 0));
                rightShooterMotor.setPower(flywheelController.calculate(targetVelocity - velocity, targetVelocity, 0));

            } else {
                targetVelocity = 0;
                leftShooterMotor.setPower(0.0);
                rightShooterMotor.setPower(0.0);
            }

            //hold left bumper to spin, then press the right bumper to shoot
            if (gamepad2.left_bumper) {
                correctedPower = desiredPower * (nominalVoltage / Math.max(batteryVoltage, 1.0));
                correctedPower = Math.max(-1.0, Math.min(correctedPower, 1.0));
                intakeMotor.setPower(correctedPower);
            } else {
                intakeMotor.setPower(0.0);
            }

            if (gamepad2.left_trigger > 0.1) {
                transferServo.setPower(0.5);
            } else {
                transferServo.setPower(0);
            }

            if (gamepad2.dpad_up) {
                leftAxonHood.setPower(1);
                rightAxonHood.setPower(1);
            } else {
                leftAxonHood.setPower(0);
                rightAxonHood.setPower(0);
            }

            if (gamepad2.dpad_down) {
                leftAxonHood.setPower(-1);
                rightAxonHood.setPower(-1);
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
}