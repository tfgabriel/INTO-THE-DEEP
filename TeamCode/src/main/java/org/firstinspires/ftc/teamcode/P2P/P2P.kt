package org.firstinspires.ftc.teamcode.P2P

import org.firstinspires.ftc.teamcode.ALGORITHMS.Math.ang_diff
import org.firstinspires.ftc.teamcode.ALGORITHMS.PDF
import org.firstinspires.ftc.teamcode.ALGORITHMS.Pose
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.chassis
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.imew
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.localizer
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.angular_tolerance
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.current_pos
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.ep
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.err
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.hPDF
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.h_d
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.h_err
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.h_f
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.h_p
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.path
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.slow
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.tolerance
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.xPDF
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.x_d
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.x_err
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.x_f
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.x_p
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.yPDF
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.y_d
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.y_err
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.y_f
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.y_p
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sign
import kotlin.math.sin

class P2P {

    fun followpath(current_path: Pose){
        xPDF = PDF(x_p, x_d, x_f)
        yPDF = PDF(y_p, y_d, y_f)
        hPDF = PDF(h_p, h_d, h_f)

        path = current_path
    }

    fun isBotinTolerance() = err.distance() < tolerance && abs(h_err) < angular_tolerance

    fun update() {
        current_pos = Pose(localizer.position.x, localizer.position.y, if(localizer.position.h >= 0.0) localizer.position.h else 2 * PI + localizer.position.h )
        err = path - current_pos

        y_err = cos(imew.yaw) * (err.x) - sin(imew.yaw) * (err.y)
        x_err = sin(imew.yaw) * (err.x) + cos(imew.yaw) * (err.y)
        h_err = ang_diff(path.h, imew.yaw)

        // If the robot is not in tolerance, run with the pd
        if (!isBotinTolerance()) {
            chassis.rc_drive( -xPDF.update(x_err), -yPDF.update(y_err) * slow, -hPDF.update(h_err), 0.0)
            ep.reset()
        }
        else {
            // else, run with the pd in the opposite direction in order to stop it and counteract the slip for a bit then stop
            if(ep.milliseconds()<50)
                chassis.rc_drive( y_f * sign(y_err),  x_f * sign(x_err), h_f * sign(h_err), 0.0)
            else
                chassis.rc_drive(0.0, 0.0, 0.0, 0.0)
        }

    }
}