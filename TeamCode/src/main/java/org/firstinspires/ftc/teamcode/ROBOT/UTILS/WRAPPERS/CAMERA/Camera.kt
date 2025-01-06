package org.firstinspires.ftc.teamcode.ROBOT.UTILS.WRAPPERS.CAMERA

import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import org.firstinspires.ftc.teamcode.ALGORITHMS.VecVec2D
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.hardwareMap

class Camera {
    var limelight: Limelight3A = hardwareMap.get(Limelight3A::class.java, "limelight");

    val result: LLResult?
        get(){
            return  limelight.latestResult
        }

    val ang_X: Double
        get(){
            return if(result != null)
                result!!.tx
            else
                0.0
        }

    val ang_Y: Double
        get(){
            return if(result != null)
                result!!.ty
            else
                0.0
        }

    val is_open: Boolean
        get(){
            return limelight.isRunning
        }

    val targetCorners: MutableList<MutableList<Double>>?
        get(){
            return if(result != null)
            result!!.colorResults[0].targetCorners
            else
                null
        }

    fun corners(): VecVec2D? {
        return if(result!= null && targetCorners!= null)
            VecVec2D(result!!.colorResults[0].targetCorners)
        else null
    }

}