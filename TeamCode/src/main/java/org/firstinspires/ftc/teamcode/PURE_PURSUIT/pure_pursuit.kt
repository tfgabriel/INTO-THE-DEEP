package org.firstinspires.ftc.teamcode.PURE_PURSUIT

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.ALGORITHMS.Intersection
import org.firstinspires.ftc.teamcode.ALGORITHMS.Math.ang_diff
import org.firstinspires.ftc.teamcode.ALGORITHMS.Math.interpolate_heading
import org.firstinspires.ftc.teamcode.ALGORITHMS.PDF
import org.firstinspires.ftc.teamcode.ALGORITHMS.Path
import org.firstinspires.ftc.teamcode.ALGORITHMS.Pose
import org.firstinspires.ftc.teamcode.ALGORITHMS.Trajectory
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
import org.firstinspires.ftc.teamcode.PURE_PURSUIT.pp_vars.robot_radius
import org.firstinspires.ftc.teamcode.TELEMETRY.communication.send_toall
import kotlin.math.abs
import kotlin.math.sign


object pp_vars{
    @JvmField
    var robot_radius: Double = 32.0
}



class pure_pursuit() {

    var curr_pos = Pose()
    var target_pos = Pose()
    var last_pos = Pose()

    var curr_traj = Trajectory()
    var istangent = false

    var xPDF: PDF = PDF()
    var yPDF: PDF = PDF()
    var hPDF: PDF = PDF()

    var err = Pose()
    val ep: ElapsedTime = ElapsedTime()
    var isdone: Boolean = false

    var traj_index: Int = 0

    var target_heading: Double = interpolate_heading(curr_traj[traj_index].sp.h,
        curr_traj[traj_index].ep.h,
        (curr_traj[traj_index].ep - curr_traj[traj_index].sp).distance())


    fun getIntersection(path: Path, center: Pose, radius: Double): Pose {
        val intersect = Intersection(path, center, radius)
        val discriminant = intersect.discriminant()

        val target: Pose
        if(!isAtEndOfPath()) {
            if (discriminant < 0) {
                send_toall("robot", "IS OFF TRACK !!!!!")
                target = last_pos
                istangent = false
            } else if (discriminant == 0.0) {
                send_toall("robot", "IS TANGENT !!!!!")
                target = if(intersect.only_solution() != intersect.exception)
                    Pose(intersect.only_solution(), target_heading, path.sp.vel)
                else
                    last_pos

                istangent = true
            } else {
                send_toall("robot", "IS SECANT !!!!!")
                target = if(intersect.closest_solution() != intersect.exception)
                    Pose(intersect.closest_solution(), target_heading, path.sp.vel)
                else
                    last_pos
                istangent = true
            }
        }
        else{
            target = path.ep
            istangent = true
        }

        return target
    }

    fun getIntersection(trajectory: Trajectory, center: Pose, radius: Double): Pose {

        if(traj_index <= trajectory.size) {
            val new_target = getIntersection(trajectory[traj_index+1], center, radius)
            if(istangent) {
                traj_index++
                return new_target
            }
        }

        return getIntersection(trajectory[traj_index+1], center, radius)
    }

    fun isBotinTolerance() = err.distance() < tolerance && abs(err.h) < angular_tolerance

    fun isAtEndOfPath() = curr_traj[traj_index].ep.distance() <= robot_radius* robot_radius


    fun followpath(traj: Trajectory) {
        xPDF = PDF(x_p, x_d, x_f)
        yPDF = PDF(y_p, y_d, y_f)
        hPDF = PDF(h_p, h_d, h_f)

        curr_traj = traj
        isdone = false
    }

    fun update() {
        curr_pos = localizer.pose + Pose(0.0, 0.0, 0.0, target_pos.vel)
        target_pos = getIntersection(curr_traj, curr_pos, robot_radius)
        last_pos = target_pos

        err = target_pos - curr_pos

        err = err.rotate(-curr_pos.h)
        err.h = ang_diff(curr_pos.h, target_pos.h)


        if (!isBotinTolerance()) {
            chassis.rc_drive(
                -yPDF.update(err.y) * target_pos.vel,
                xPDF.update(-err.x) * target_pos.vel,
                -hPDF.update(err.h),
                0.0
            )
            ep.reset()
        }
        else {
            // else, run with the pd in the opposite direction in order to stop it and counteract the slip for a bit then stop
            if(ep.milliseconds()<10)
                chassis.rc_drive( y_f * sign(err.y),  -x_f * sign(err.x), h_f * sign(err.h), 0.0)
            else
                chassis.rc_drive(0.0, 0.0, 0.0, 0.0)
        }
    }
}
