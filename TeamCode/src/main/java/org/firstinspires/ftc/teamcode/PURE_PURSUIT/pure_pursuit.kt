package org.firstinspires.ftc.teamcode.PURE_PURSUIT

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.ALGORITHMS.Math.angNorm
import org.firstinspires.ftc.teamcode.ALGORITHMS.Math.ang_diff
import org.firstinspires.ftc.teamcode.ALGORITHMS.PDF
import org.firstinspires.ftc.teamcode.ALGORITHMS.Path
import org.firstinspires.ftc.teamcode.ALGORITHMS.Point
import org.firstinspires.ftc.teamcode.ALGORITHMS.Pose
import org.firstinspires.ftc.teamcode.AUTO.SpecimenVars
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.chassis
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.localizer
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.angular_tolerance
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.h_d
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.h_f
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.h_p
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.tolerance
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.x_d
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.x_f
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.x_p
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.y_d
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.y_f
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.y_p
import org.firstinspires.ftc.teamcode.PURE_PURSUIT.pp_vars.A
import org.firstinspires.ftc.teamcode.PURE_PURSUIT.pp_vars.B
import org.firstinspires.ftc.teamcode.PURE_PURSUIT.pp_vars.C
import org.firstinspires.ftc.teamcode.PURE_PURSUIT.pp_vars.alpha
import org.firstinspires.ftc.teamcode.PURE_PURSUIT.pp_vars.curr_path
import org.firstinspires.ftc.teamcode.PURE_PURSUIT.pp_vars.curr_pos
import org.firstinspires.ftc.teamcode.PURE_PURSUIT.pp_vars.d
import org.firstinspires.ftc.teamcode.PURE_PURSUIT.pp_vars.discriminant
import org.firstinspires.ftc.teamcode.PURE_PURSUIT.pp_vars.dist
import org.firstinspires.ftc.teamcode.PURE_PURSUIT.pp_vars.dist_circle
import org.firstinspires.ftc.teamcode.PURE_PURSUIT.pp_vars.dist_from_center
import org.firstinspires.ftc.teamcode.PURE_PURSUIT.pp_vars.ep
import org.firstinspires.ftc.teamcode.PURE_PURSUIT.pp_vars.err
import org.firstinspires.ftc.teamcode.PURE_PURSUIT.pp_vars.hPDF
import org.firstinspires.ftc.teamcode.PURE_PURSUIT.pp_vars.isdone
import org.firstinspires.ftc.teamcode.PURE_PURSUIT.pp_vars.last_pose
import org.firstinspires.ftc.teamcode.PURE_PURSUIT.pp_vars.pos_tolerance
import org.firstinspires.ftc.teamcode.PURE_PURSUIT.pp_vars.robot_radius
import org.firstinspires.ftc.teamcode.PURE_PURSUIT.pp_vars.slope
import org.firstinspires.ftc.teamcode.PURE_PURSUIT.pp_vars.target_pos
import org.firstinspires.ftc.teamcode.PURE_PURSUIT.pp_vars.xPDF
import org.firstinspires.ftc.teamcode.PURE_PURSUIT.pp_vars.yPDF
import kotlin.math.abs
import kotlin.math.sign
import kotlin.math.sqrt


object pp_vars{
    @JvmField
    var robot_radius: Double = 32.0

    var curr_pos = Pose()
    var target_pos = Pose()

    var dist: Double = 0.0
    var dist_from_center: Double = 0.0
    var dist_circle: Double = 0.0
    var discriminant: Double = 0.0
    var slope: Double = 0.0
    var d = Pose()
    var alpha: Double = 0.0
    var A: Double = 0.0
    var B: Double = 0.0
    var C: Double = 0.0
    var err = Pose()
    var curr_path = Path()
    var isdone: Boolean = false
    var last_pose = Pose()

    var xPDF: PDF = PDF()
    var yPDF: PDF = PDF()
    var hPDF: PDF = PDF()
    val ep: ElapsedTime = ElapsedTime()
    var done: Boolean = false

    var pos_tolerance = Pose(3.0, 3.0, 0.18)
}



class pure_pursuit() {

    fun getIntersection(path: Path, center: Pose, radius: Double): Pose {
        d = path.ep - path.ep
        slope = d.y / d.x
        alpha = -slope * path.sp.x + path.sp.y - center.y

        A = 1 + slope * slope
        B = 2 * (1 + slope * alpha)
        C = alpha * alpha + radius * radius

        discriminant = B * B - 4 * A * C

        if (discriminant < 0.0)
            return last_pose
        else if (discriminant == 0.0) {
            val x = B / (2 * A)
            val y = slope * (x + path.sp.x) + path.sp.y
            return Pose(x, y)
        } else {
            val x1 = (B - sqrt(discriminant)) / 2 * A
            val x2 = (B + sqrt(discriminant)) / 2 * A

            val y1 = slope * (x1 + path.sp.x) + path.sp.y
            val y2 = slope * (x2 + path.sp.x) + path.sp.y

            val p1 = Pose(x1, y1)
            val p2 = Pose(x2, y2)

            if ((path.ep - p1).distance() > (path.ep - p2).distance()) {
                last_pose = p2
                return p2
            } else {
                last_pose = p1
                return p1
            }
        }

    }

    fun isBotinTolerance() = err.distance() < tolerance && abs(err.h) < angular_tolerance

    fun isPoseinTolerance() =
        abs(curr_path.ep.x - curr_pos.x) < pos_tolerance.x &&
                abs(curr_path.ep.y - curr_pos.y) < pos_tolerance.y &&
                abs(ang_diff(curr_path.ep.h, curr_pos.h)) < pos_tolerance.h


    fun followpath(path: Path) {
        xPDF = PDF(x_p, x_d, x_f)
        yPDF = PDF(y_p, y_d, y_f)
        hPDF = PDF(h_p, h_d, h_f)

        curr_path = path
        isdone = false
    }

    fun update() {
        curr_pos = Pose(localizer.pose.x, localizer.pose.y, angNorm(localizer.pose.h))
        target_pos = getIntersection(curr_path, curr_pos, robot_radius)

        err = target_pos - curr_pos

        err = err.rotate(-curr_pos.h)
        err.h = ang_diff(curr_pos.h, target_pos.h)

        if (!isBotinTolerance()) {
            chassis.rc_drive(
                -yPDF.update(err.y) * SpecimenVars.slow,
                xPDF.update(-err.x) * SpecimenVars.slow,
                -hPDF.update(err.h),
                0.0
            )
            ep.reset()
        } else if (!isBotinTolerance()) {
            target_pos = getIntersection(curr_path, curr_pos, robot_radius)
        } else if (isPoseinTolerance()) {
            if (ep.milliseconds() < 50)
                chassis.rc_drive(-y_f * sign(err.y), x_f * sign(err.x), h_f * sign(err.h), 0.0)
            else
                chassis.rc_drive(0.0, 0.0, 0.0, 0.0)
        }
    }
}
