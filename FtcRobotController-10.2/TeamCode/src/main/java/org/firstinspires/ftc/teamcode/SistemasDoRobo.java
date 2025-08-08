package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class SistemasDoRobo {
    Braco bracoColeta;
    Elevador elevador;
    Cesta cesta;
    Garras garras;
    long temporizadorMovimento;
    int delay;
    public SistemasDoRobo(HardwareMap hm, boolean resetaEncoderDoBraco, boolean resetaEncoderDoElevador) {

        //DECLARANDO O BRAÇO DA COLETA E RESETANDO ENCODER
        bracoColeta = new Braco(hm);
        if (resetaEncoderDoBraco) {
            bracoColeta.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bracoColeta.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        //DECLARANDO O ELEVADOR
        elevador = new Elevador(hm);
        if (resetaEncoderDoElevador) {
            elevador.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        elevador.DescerTotal();

        cesta = new Cesta(hm);
        cesta.posicaoInicial();

        garras = new Garras(hm);
    }

    public void baixarBracoColeta() {
        bracoColeta.setTargetTicks(280); //QUANDO ENCERRA O LOOPING É PORQUE JÁ BAIXOU O BRAÇO ATÉ A POSIÇÃO DE COLETA
        //bracoColeta.update();
        temporizadorMovimento = System.currentTimeMillis();
    }

    public void subirBracoTransferencia() {
        bracoColeta.setTargetTicks(20); //QUANDO ENCERRA O LOOPING É PORQUE JÁ SUBIU O BRAÇO ATÉ A POSIÇÃO DE TRANSFERÊNCIA
        bracoColeta.update();
        temporizadorMovimento = System.currentTimeMillis();
    }

    public void bracoMeiaAltura() { //QUANDO ENCERRA O LOOPING É PORQUE JÁ DEIXOU O BRAÇO A MEIA ALTURA
        bracoColeta.setTargetTicks(150);
        bracoColeta.update();
        temporizadorMovimento = System.currentTimeMillis();
    }

    public void depositar() { //SOBE E PONTUA
        bracoMeiaAltura();
        elevador.SubirPraCestaAlta();
        sleep(250);
        while (!elevador.atTarget()) {
        }
        cesta.Depositar();
        elevador.DescerTotal();
        sleep(250);
    }

    public void update() {
        bracoColeta.update();
        elevador.Executar();
        garras.updateModulacao();
    }

    public void sleep(long tempo) {
        long temporizadorSleep = System.currentTimeMillis();
        while (System.currentTimeMillis() - temporizadorSleep < tempo) {update();}
    }

    public void coletarSample() {
        delay = 200;
        garras.modularPraFora();

        sleep(delay);

        baixarBracoColeta();
        while (!bracoColeta.atTarget() && (System.currentTimeMillis() - temporizadorMovimento) < 1000) {
            update();
        }

        sleep(delay);

        garras.fecharPinca();

        sleep(200);

        garras.modularPraDentro();
        
        sleep(200);

        subirBracoTransferencia();
        while (!bracoColeta.atTarget() && (System.currentTimeMillis() - temporizadorMovimento) < 1000) {
            update();
        }

        sleep(delay);

        garras.abrirPinca();

        sleep(delay);

        bracoMeiaAltura();

        //telemetry.addData("Status coleta", "Já COLOQUEI BRAÇO MEIA ALTURA");
        //telemetry.update();

        while (!bracoColeta.atTarget() && (System.currentTimeMillis() - temporizadorMovimento) < 1000) {
            update();
        }
    }
}
