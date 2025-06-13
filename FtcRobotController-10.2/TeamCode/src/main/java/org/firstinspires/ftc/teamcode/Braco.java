package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Braco {
    private DcMotorEx motor;

    private PIDFController pid;
    private double targetTicks = 0;

    PIDCoefficients constantesPID = new PIDCoefficients(0.01, 0, 0);

    public static double kF = 0.0;

    public Braco(HardwareMap hm, boolean resetEncoder) {
        motor = hm.get(DcMotorEx.class, "Braco");
        if (resetEncoder) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pid = new PIDFController(constantesPID, kF);
        pid.setInputBounds(-1000, 0); // Exemplo: braço se move entre ticks negativos
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
