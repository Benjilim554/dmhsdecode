package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.JoinedTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystem.util.PIDFController;

import com.bylazar.configurables.annotations.Configurable;

/*

1) kS stands for "static." It's the greatest power you can give the flywheel motor(s) without it moving.
So it should be a really small number below like 0.05 or something.
You should increase it until it begins to move and then decrease it until it stops moving again.
2) kV is the next number you tune. Pick a velocity like 1500 or something and then increase kV until it
gets there.
3) Tune P. Send a ball through the launcher and increase P until it doesn't overshoot past the target
velocity in the recovery phase. Keep increasing though because unless you see overshoot you're fine.
4) Tune I. After all the other numbers are tuned, save them, and then make kS slightly negative.
This will simulate a drop in power to the motor from a low battery. Increase I until your confident
that you can still get to your target velocity.

 */

@Configurable
@TeleOp
public class FlywheelTuning extends OpMode {
    private PIDFController controller;
    private DcMotorEx encoder;
    private DcMotorEx leftShooterMotor, rightShooterMotor;
    public static double targetVelocity, encoderVelocity, velocity;
    public static double P, I , kV, kA , kS;
    @Override
    public void init() {
        //TODO: Set motor name and direction
        telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

        encoder = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightHoodMotor");
        leftShooterMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftHoodMotor"); // change directions if needed
        rightShooterMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightHoodMotor"); // change directions if needed

        encoder.setDirection(DcMotorSimple.Direction.FORWARD);
        rightShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        controller = new PIDFController(P, I, 0.0, 0.0, kV, 0.0, 0.1338); // PIDF, Feedforward(kV, kA, kS)
    }

    @Override
    public void loop() {
        telemetry.addLine("==Values==");
        telemetry.addData("TargetVel: ", targetVelocity);
        telemetry.addData("CurrentVel: ", velocity);

        telemetry.addLine("==========");

        controller.setPIDF(P, I, 0.0, 0);
        controller.setFeedforward(kV, 0, kS);

        velocity = encoder.getVelocity();

        leftShooterMotor.setPower(controller.calculate((targetVelocity - velocity), targetVelocity, 0));
        rightShooterMotor.setPower(controller.calculate((targetVelocity - velocity), targetVelocity, 0));
    }
}