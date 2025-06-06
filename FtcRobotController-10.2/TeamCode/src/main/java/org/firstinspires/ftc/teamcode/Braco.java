package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Disabled
public class Braco {
    DcMotor modularBraco;
    public Braco(HardwareMap hardwareMap, boolean resetaEncoder) {
        modularBraco = hardwareMap.dcMotor.get("Braco");

        if (resetaEncoder) {
            modularBraco.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        modularBraco.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void moverBraco(int posicao) {

    }
}
