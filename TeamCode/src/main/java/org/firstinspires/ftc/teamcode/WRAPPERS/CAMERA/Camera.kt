package org.firstinspires.ftc.teamcode.WRAPPERS.CAMERA

import com.qualcomm.hardware.limelightvision.Limelight3A
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.hardwareMap

class Camera {
    var limelight: Limelight3A = hardwareMap.get(Limelight3A::class.java, "limelight");

    val ang_X: Double
        get(){
            return limelight.latestResult.tx
        }

    val ang_Y: Double
        get(){
            return limelight.latestResult.ty
        }
}