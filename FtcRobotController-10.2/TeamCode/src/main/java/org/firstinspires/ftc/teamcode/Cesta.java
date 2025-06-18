package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Cesta {
    Servo cesta;
    long temporizadorCesta;

    public Cesta(HardwareMap hm) {
        cesta = hm.servo.get("Cesta");
    }

    public void posicaoInicial() {
        cesta.setPosition(1);
    }

    public void Depositar() {
        temporizadorCesta = System.currentTimeMillis();
        cesta.setPosition(0.3);
        while ((System.currentTimeMillis() - temporizadorCesta) < 700) {}
        cesta.setPosition(0.85);
    }
}
