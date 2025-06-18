package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Elevador { //FALTA DESCOBRIR OS VALORES CERTINHOS DE CADA POSIÇÃO COM A REDUÇÃO NOVA
    DcMotor elevador;

    public void Executar() {
        elevador.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public Elevador(HardwareMap hm) {
        elevador = hm.dcMotor.get("ExpansaoV");
        elevador.setDirection(DcMotorSimple.Direction.REVERSE);
        elevador.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevador.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevador.setPower(1);
    }

    public void SubirPraCestaAlta() {
        elevador.setTargetPosition(3000);
        Executar();
    }

    public void SubirPraCestaBaixa() {
        elevador.setTargetPosition(2000);
        Executar();
    }

    public void DescerTotal() {
        elevador.setTargetPosition(3000);
        Executar();
    }

    public void DescerPraColetarEspecime() {
        elevador.setTargetPosition(500);
        Executar();
    }

    public void SubirTrelicaAlta() {
        elevador.setTargetPosition(2500);
        Executar();
    }

    public void SubirTrelicaBaixa() {
        elevador.setTargetPosition(1000);
        Executar();
    }
}
