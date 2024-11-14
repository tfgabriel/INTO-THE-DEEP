package org.firstinspires.ftc.teamcode.P2P

import com.qualcomm.robotcore.util.ElapsedTime
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
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.hPDFL
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.h_d
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.h_err
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.h_f
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.h_p
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.path
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.slow
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.tolerance
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.xPDFL
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.x_d
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.x_err
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.x_f
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.x_p
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.yPDFL
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.y_d
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.y_err
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.y_f
import org.firstinspires.ftc.teamcode.P2P.p2p_vars.y_p
import kotlin.math.cos
import kotlin.math.sign
import kotlin.math.sin

class P2P {

    fun followpath(current_path: Pose){
        xPDFL = PDF(x_p, x_d, x_f)
        yPDFL = PDF(y_p, y_d, y_f)
        hPDFL = PDF(h_p, h_d, h_f)

        path = current_path
    }

    fun isBotinTolerance() = err.distance() < tolerance && h_err < angular_tolerance

    fun update() {
        current_pos = localizer.pos
        err = path - current_pos

        y_err = cos(imew.yaw) * (err.x) - sin(imew.yaw) * (err.y)
        x_err = sin(imew.yaw) * (err.x) + cos(imew.yaw) * (err.y)
        h_err = ang_diff(current_pos.h, imew.yaw)

        if (!isBotinTolerance()) {
            chassis.fc_drive(yPDFL.update(y_err) * slow, xPDFL.update(x_err) * slow, hPDFL.update(h_err), 0.0)
            ep.reset()
        }
        else {
            if(ep.milliseconds()<50)
                chassis.fc_drive(- y_f * sign(y_err), - x_f * sign(x_err),- h_f * sign(h_err), 0.0)
            else
                chassis.fc_drive(0.0, 0.0, 0.0, 0.0)
        }

    }
}