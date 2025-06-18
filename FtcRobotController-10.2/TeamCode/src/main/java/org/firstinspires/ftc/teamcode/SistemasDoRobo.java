package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SistemasDoRobo {
    Braco bracoColeta;
    Elevador elevador;
    Cesta cesta;
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
            elevador.elevador.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        cesta = new Cesta(hm);
    }
}
