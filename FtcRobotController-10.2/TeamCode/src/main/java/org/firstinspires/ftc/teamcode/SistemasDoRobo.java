package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SistemasDoRobo {
    Braco bracoColeta;
    Elevador elevador;
    Cesta cesta;
    Garras garras;
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

        cesta = new Cesta(hm);
        cesta.posicaoInicial();

        garras = new Garras(hm);
    }

    public void baixarBracoColeta() {
        bracoColeta.setTargetTicks(350); //QUANDO ENCERRA O LOOPING É PORQUE JÁ BAIXOU O BRAÇO ATÉ A POSIÇÃO DE COLETA
        bracoColeta.update();
    }

    public void subirBracoTransferencia() {
        bracoColeta.setTargetTicks(0); //QUANDO ENCERRA O LOOPING É PORQUE JÁ SUBIU O BRAÇO ATÉ A POSIÇÃO DE TRANSFERÊNCIA
        bracoColeta.update();
    }

    public void bracoMeiaAltura() { //QUANDO ENCERRA O LOOPING É PORQUE JÁ DEIXOU O BRAÇO A MEIA ALTURA
        bracoColeta.setTargetTicks(180);
        bracoColeta.update();
    }

    public void depositar() { //SOBE E PONTUA
        bracoMeiaAltura();
        elevador.SubirPraCestaAlta();
        while (elevador.motor.isBusy()) {
        }
        cesta.Depositar();
        elevador.DescerTotal();
    }

    public void update() {
        bracoColeta.update();
        elevador.Executar();
        garras.updateModulacao();
    }

    public void coletarSample() {
        garras.modularPraFora();
        baixarBracoColeta();
        while (!bracoColeta.atTarget()) {}
        garras.fecharPinca();
        garras.modularPraDentro();
        subirBracoTransferencia();
        while (!bracoColeta.atTarget()) {}
        garras.abrirPinca();
        bracoMeiaAltura();
        while (!bracoColeta.atTarget()) {}
    }
}
