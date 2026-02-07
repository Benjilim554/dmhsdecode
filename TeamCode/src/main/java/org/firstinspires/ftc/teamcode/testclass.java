package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Android Studio Test")
public class testclass extends LinearOpMode {
    private DcMotorEx leftShooterMotor, rightShooterMotor, intakeMotor;
    private DcMotorEx leftBackMotor, leftFrontMotor, rightBackMotor, rightFrontMotor;
    private CRServo transferServo, leftAxonHood, rightAxonHood;

    private double rx;
    private double ctrlPow = 2.0;


    @Override
    public void runOpMode() {
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

        rx = (gamepad1.right_stick_x / 4) * 3;

        rightFrontMotor.setPower(Math.pow(gamepad1.left_stick_y + gamepad1.left_stick_x + rx, ctrlPow) * Math.signum(gamepad1.left_stick_y + gamepad1.left_stick_x + rx));
        leftFrontMotor.setPower(Math.pow(gamepad1.left_stick_y - gamepad1.left_stick_x - rx, ctrlPow) * Math.signum(gamepad1.left_stick_y - gamepad1.left_stick_x - rx));
        rightBackMotor.setPower(Math.pow(gamepad1.left_stick_y - gamepad1.left_stick_x + rx, ctrlPow) * Math.signum(gamepad1.left_stick_y - gamepad1.left_stick_x + rx));
        leftBackMotor.setPower(Math.pow(gamepad1.left_stick_y + gamepad1.left_stick_x - rx, ctrlPow) * Math.signum(gamepad1.left_stick_y + gamepad1.left_stick_x - rx));

        waitForStart();
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
            leftShooterMotor.setPower(1);
            rightShooterMotor.setPower(1);
            //leftShooterMotor.setPower(flywheelController.calculate(targetVelocity - velocity, targetVelocity, 0));
            // rightShooterMotor.setPower(flywheelController.calculate(targetVelocity - velocity, targetVelocity, 0));

        } else {
            leftShooterMotor.setPower(0.0);
            rightShooterMotor.setPower(0.0);
        }
        if (gamepad2.right_bumper) {

            leftShooterMotor.setPower(-1);
            rightShooterMotor.setPower(-1);


        } else {

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


    }
    }
