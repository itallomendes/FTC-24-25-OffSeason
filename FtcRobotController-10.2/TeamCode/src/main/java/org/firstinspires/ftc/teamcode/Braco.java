package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Disabled
public class Braco {
    DcMotor modularBraco;

    public Braco(HardwareMap hardwareMap) {
        modularBraco = hardwareMap.dcMotor.get("Braco");
    }

    public void moverBraco(int posicao) {
        modularBraco.setTargetPosition(posicao);
        modularBraco.setPower(0.7);
        modularBraco.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
