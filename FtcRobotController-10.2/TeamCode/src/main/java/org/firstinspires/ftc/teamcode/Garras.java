package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Garras {
    CRServo pinca, garra;
    Servo servoModularGarraVerticalD, servoModularGarraVerticalE;
    double posicaoVerticalGarra = 0.1; //GARRA COMEÇA PRA FORA DA CESTA
    public Garras(HardwareMap hm) {
        servoModularGarraVerticalE = hm.get(Servo.class, "ModularE");
        servoModularGarraVerticalD = hm.get(Servo.class, "ModularD");

        updateModulacao();

        pinca = hm.crservo.get("Pinca");
        garra = hm.crservo.get("Garra");
    }

    public void updateModulacao() {
        servoModularGarraVerticalD.setPosition(posicaoVerticalGarra);
        servoModularGarraVerticalE.setPosition(1-posicaoVerticalGarra);
    }

    public void abrirPinca() {
        long temporizadorDaPinca = System.currentTimeMillis();
        pinca.setPower(0.5);
        while (System.currentTimeMillis() - temporizadorDaPinca < 500) {
        }
        pinca.setPower(0);
    }

    public void fecharPinca() {
        long temporizadorDaPinca = System.currentTimeMillis();
        pinca.setPower(-0.5);
        while (System.currentTimeMillis() - temporizadorDaPinca < 800) {
        }
        pinca.setPower(0);
    }

    public void abrirGarra() {
        long temporizadorDaGarra = System.currentTimeMillis();
        garra.setPower(0.5);
        while (System.currentTimeMillis() - temporizadorDaGarra < 500) {
        }
        garra.setPower(0);
    }

    public void fecharGarra() {
        long temporizadorDaGarra = System.currentTimeMillis();
        garra.setPower(-0.5);
        while (System.currentTimeMillis() - temporizadorDaGarra < 500) {
        }
        garra.setPower(0);
    }

    public void modularPraFora() {
        posicaoVerticalGarra = 0.1;
        updateModulacao();
    }

    public void modularPraDentro() {
        posicaoVerticalGarra = 0.8;
        updateModulacao();
    }
}
