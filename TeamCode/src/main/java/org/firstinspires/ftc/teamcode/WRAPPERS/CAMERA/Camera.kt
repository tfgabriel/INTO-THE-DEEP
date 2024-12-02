package org.firstinspires.ftc.teamcode.WRAPPERS.CAMERA

import com.qualcomm.hardware.limelightvision.Limelight3A
import org.firstinspires.ftc.teamcode.ALGORITHMS.Point
import org.firstinspires.ftc.teamcode.ALGORITHMS.PointVec
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.hardwareMap
import org.firstinspires.ftc.teamcode.WRAPPERS.CAMERA.camera_vars.angtopos

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

    val is_open: Boolean
        get(){
            return limelight.isRunning
        }

    val targetCorners: List<List<Double>>
        get(){
            return limelight.latestResult.colorResults[0].targetCorners
        }

    fun corners(): PointVec{
        return PointVec(Point(targetCorners[0][0], targetCorners[0][1]), Point(targetCorners[1][0], targetCorners[1][1]), Point(targetCorners[2][0], targetCorners[2][1]), Point(targetCorners[3][0], targetCorners[3][1]))
    }

}