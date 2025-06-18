package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


@Config
public class Braco {
    public DcMotorEx motor;
    private PIDFController pid;
    private double targetTicks = 0;

    PIDCoefficients constantesPID = new PIDCoefficients(0.08, 0.0002, 0.0003);

    public static double kF = 0.13;

    public Braco(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "modularcoleta");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pid = new PIDFController(constantesPID, kF);
        pid.setInputBounds(0, 500); // Exemplo: braço se move entre ticks negativos
        pid.setOutputBounds(-1, 1);
    }

    public void setTargetTicks(double ticks) {
        targetTicks = ticks;
        pid.setTargetPosition(ticks);
    }

    public void update() {
        double current = motor.getCurrentPosition();
        double power = pid.update(current);
        motor.setPower(power);
    }

    public boolean atTarget() {
        return Math.abs(motor.getCurrentPosition() - targetTicks) < 10;
    }
}
