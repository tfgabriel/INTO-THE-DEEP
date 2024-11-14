package org.firstinspires.ftc.teamcode.TELEMETRY

import android.annotation.SuppressLint
import org.firstinspires.ftc.teamcode.BOT_CONFIG.robot_vars.telemetry

@SuppressLint("DefaultLocale")
object communication {
    fun send_todash(caption: String, content: Any){
        telemetry.addData(caption, content)
    }

    fun send_todash(caption: String, content: Double) = send_todash(caption, String.format("%.4f", content))
    fun send_todash(caption: String, content: Float) = send_todash(caption, String.format("%.4f", content))

    fun send_tods(caption: String, content: Any){
        telemetry.addLine(caption + content)
        telemetry.update()
    }

    fun send_tods(caption: String, content: Double) = send_tods(caption, String.format("%.4f", content))
    fun send_tods(caption: String, content: Float) = send_tods(caption, String.format("%.4f", content))

    fun send_toall(caption: String, content: Any){
        send_tods(caption, content)
        send_todash(caption, content)
    }
}