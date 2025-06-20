package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class PIDF_braco extends OpMode {
    PIDController controle;
    public static double p = 0.08, i = 0.0002, d = 0.0003;
    public static double f = 0.13;

    public static int target = 0;

    private final double ticks_in_degree = 560 / 180.0;

    private DcMotorEx motor;

    @Override
    public void init() {
        controle = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor = hardwareMap.get(DcMotorEx.class, "modularcoleta");
    }

    @Override
    public void loop() {
        controle.setPID(p, i, d);

        int PosicaoBraco = motor.getCurrentPosition();

        double pid = controle.calculate(PosicaoBraco, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;

        motor.setPower(power);

        telemetry.addData ("pos", PosicaoBraco);
        telemetry.addData ("target", target);
        telemetry.update();
    }
}
