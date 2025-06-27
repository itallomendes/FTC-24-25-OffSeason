package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.controller.PIDController;


@Config
public class Braco {
    public DcMotorEx motor;
    public PIDController pid;
    private int targetTicks = 0;
    public int current;

    public Braco(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "modularcoleta");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        pid = new PIDController(0.035,0.2, 0.00125); //f = 0.13
    }

    public void setTargetTicks(int ticks) {
        targetTicks = ticks;
    }

    public void update() {
        pid.setPID(0.035, 0.2, 0.00125);

        current = motor.getCurrentPosition();

        double calculoPID = pid.calculate(current, targetTicks);
        double ff = Math.cos(Math.toRadians(targetTicks / (1120/360))) * 0.12;

        double power = calculoPID + ff;

        power = Math.max(Math.min(power, 0.2), -0.7);

        motor.setPower(power);
    }


    public boolean atTarget() {
        return Math.abs(motor.getCurrentPosition() - targetTicks) < 10;
    }
}
