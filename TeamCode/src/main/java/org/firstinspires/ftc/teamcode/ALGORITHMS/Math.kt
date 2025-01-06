package org.firstinspires.ftc.teamcode.ALGORITHMS

import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.camera_ang
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.camera_distance_from_ground
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.cm_to_ticks
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.servo_range
import java.lang.Math
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan
import kotlin.math.floor
import kotlin.math.hypot
import kotlin.math.tan

//a series of useful math functions i use throughout the code
object Math {
    //the mod operation for float/doubles
    fun float_mod(a: Double, b: Double) = a - floor(a / b) * b

    //difference between floats/doubles (in this code, positions) with some tolerance due to how the values are stored in memory
    fun pos_diff(pos1: Double, pos2: Double) = abs(pos1-pos2) < 0.0001

    //angle difference calculated but so that it gets the shortest arc between the 2
    fun ang_diff(ang1: Double, ang2: Double) =
        float_mod(ang2 - ang1 + PI, 2 * PI) - PI

    fun ang_norm(o1: Double) = float_mod(o1, 2 * Math.PI)

    fun clamp(v: Double, low: Double, high: Double) = v.coerceAtLeast(low).coerceAtMost(high)

    fun cam_ang_norm(angy: Double) = angy + camera_ang

    fun ang_to_pos(pt1: Vec2D, pt2: Vec2D) = atan((pt2 - pt1).y / (pt2 - pt1).x) / servo_range

    fun y_distance(angy: Double) = (tan(cam_ang_norm(angy)) * camera_distance_from_ground) / cm_to_ticks

    fun x_distance(angx: Double, angy: Double) = tan(angx) * hypot(y_distance(angy) * cm_to_ticks, camera_distance_from_ground)

    @JvmStatic
    fun floatMod(o1: Double, o2: Double): Double {
        return o1 - Math.floor(o1 / o2) * o2
    }

    @JvmStatic
    fun angNorm(o1: Double): Double {
        return floatMod(o1, 2 * Math.PI)
    }

    @JvmStatic
    fun angDiff(o1: Double, o2: Double): Double {
        return floatMod(o2 - o1 + Math.PI, Math.PI * 2) - Math.PI
    }

    @JvmStatic
    fun interpolate_heading(h1: Double, h2: Double, dist: Double) = ang_norm(h1 + ang_diff(h1, h2) / dist)
}