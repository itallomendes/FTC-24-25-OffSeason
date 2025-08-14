package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Elevador { //FALTA DESCOBRIR OS VALORES CERTINHOS DE CADA POSIÇÃO COM A REDUÇÃO NOVA
    DcMotor motor;

    public void Executar() {
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public Elevador(HardwareMap hm) {
        motor = hm.dcMotor.get("ExpansaoV");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(1);
    }

    public void SubirPraCestaAlta() {
        motor.setTargetPosition(3850);
        Executar();
    }

    public void SubirPraCestaBaixa() {
        motor.setTargetPosition(2000);
        Executar();
    }

    public void DescerTotal() {
        motor.setTargetPosition(0);
        Executar();
    }

    public void DescerPraColetarEspecime() {
        motor.setTargetPosition(300);
        Executar();
    }

    public void SubirTrelicaAlta() {
        motor.setTargetPosition(2700);
        Executar();
    }

    public void Descerpontuar() {
        motor.setTargetPosition(1600);
        Executar();
    }

    public boolean atTarget() {
        return Math.abs(motor.getCurrentPosition() - motor.getTargetPosition()) < 10;
    }

}
