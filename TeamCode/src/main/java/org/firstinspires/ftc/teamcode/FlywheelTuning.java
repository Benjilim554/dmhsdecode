package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystem.util.PIDFController;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
@TeleOp
public class FlywheelTuning extends OpMode {
    private PIDFController controller;
    private DcMotorEx motor;
    public static double targetVelocity, velocity;
    public static double P, I ,kV , kS;
    @Override
    public void init() {
        //TODO: Set motor name and direction
        motor = (DcMotorEx) hardwareMap.get(DcMotor.class, "motorName");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        controller = new PIDFController(P, I, 0.0, 0.0);
    }

    @Override
    public void loop() {
        telemetry.addData("TargetVel", targetVelocity);
        telemetry.addData("CurrentVel", velocity);
        controller.setPIDF(P,I, 0.0, kV * targetVelocity + kS);
        velocity = motor.getVelocity();
        motor.setPower(controller.calculate(targetVelocity - velocity));
    }
}