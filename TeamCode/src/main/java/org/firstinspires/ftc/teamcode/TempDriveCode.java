package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class TempDriveCode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        int SpeedFactor;
        int IntakeArmPos;
        boolean my_1PersonDrive;
        boolean holdingArmMotors;

        DcMotor leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        DcMotor leftBackMotor = hardwareMap.get(DcMotor.class, "leftBackMotor");
        DcMotor rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        DcMotor rightBackMotor = hardwareMap.get(DcMotor.class, "rightBackMotor");
        DcMotor intakeFlaps = hardwareMap.get(DcMotor.class, "intakeFlaps");
        DcMotor leftSpin = hardwareMap.get(DcMotor.class, "leftSpin");
        DcMotor rightSpin = hardwareMap.get(DcMotor.class, "rightSpin");

        // Reverse one of the drive motors.
        my_1PersonDrive = false;
        SpeedFactor = 1;
        holdingArmMotors = false;
        // You will have to determine which motor to reverse for your robot.
        // In this example, the right motor was reversed so that positive
        // applied power makes it move the robot in the forward direction.
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        // You will have to determine which motor to reverse for your robot.
        // In this example, the right motor was reversed so that positive
        // applied power makes it move the robot in the forward direction.
        leftBackMotor.setDirection(DcMotor.Direction.FORWARD);
        // You will have to determine which motor to reverse for your robot.
        // In this example, the right motor was reversed so that positive
        // applied power makes it move the robot in the forward direction.
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        // You will have to determine which motor to reverse for your robot.
        // In this example, the right motor was reversed so that positive
        // applied power makes it move the robot in the forward direction.
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);
        IntakeArmPos = 0;
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                // The Y axis of a joystick ranges from -1 in its topmost position to +1 in its bottommost position.
                // We negate this value so that the topmost position corresponds to maximum forward power.
                leftBackMotor.setPower(((gamepad2.left_stick_y - gamepad2.right_stick_x) - gamepad2.left_stick_x) * SpeedFactor);
                leftFrontMotor.setPower(((gamepad2.left_stick_y - gamepad2.right_stick_x) + gamepad2.left_stick_x) * SpeedFactor);
                // The Y axis of a joystick ranges from -1 in its topmost position to +1 in its bottommost position.
                // We negate this value so that the topmost position corresponds to maximum forward power.
                rightBackMotor.setPower((gamepad2.left_stick_y + gamepad2.right_stick_x + gamepad2.left_stick_x) * SpeedFactor);
                rightFrontMotor.setPower(((gamepad2.left_stick_y + gamepad2.right_stick_x) - gamepad2.left_stick_x) * SpeedFactor);
                if (gamepad1.right_bumper) {
                    intakeFlaps.setPower(-0.5);
                } else if (gamepad1.left_bumper) {
                    intakeFlaps.setPower(0.5);
                } else {
                    intakeFlaps.setPower(0);
                }
                if (gamepad1.a) {
                    leftSpin.setPower(-1);
                    rightSpin.setPower(-1);
                } else if (gamepad1.b) {
                    leftSpin.setPower(1);
                    rightSpin.setPower(1);
                } else {
                    leftSpin.setPower(0);
                    rightSpin.setPower(0);
                }
            }
        }
    }
}