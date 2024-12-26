package org.firstinspires.ftc.teamcode.ALGORITHMS

import android.annotation.SuppressLint
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import org.firstinspires.ftc.teamcode.ALGORITHMS.Math.angNorm
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

class Array(val val1: Double, val val2: Double, val val3: Double){
    constructor(val1: Double, val2: Double): this(val1, val2, 0.0)
    constructor(val1: Double): this(val1, 0.0, 0.0)
    constructor(): this(0.0,0.0, 0.0)

    operator fun get(i: Int) = when (i) {
        0 -> val1
        1 -> val2
        else -> val3
    }
}

class Pose(@JvmField var x: Double, @JvmField var y: Double, @JvmField var h: Double){
    constructor(): this(0.0, 0.0, 0.0)
    constructor(x: Double, y: Double): this(x, y, 0.0)
    constructor(sparkpos: SparkFunOTOS.Pose2D): this(sparkpos.x, sparkpos.y, sparkpos.h)

    operator fun plus(pose: Pose): Pose = Pose(x + pose.x, y + pose.y, h + pose.h)

    operator fun minus(pose: Pose): Pose = Pose(x - pose.x, y - pose.y, h - pose.h)

    operator fun times(a: Double): Pose = Pose(a * x, a * y, a * h)


    fun rotate(angle: Double) = Pose(x * cos(angle) - y * sin(angle),
                                     x * sin(angle) + y * cos(angle),
                                        angNorm(h + angle))

    fun distance(): Double = sqrt(x*x + y*y)

    @SuppressLint("DefaultLocale")
    override fun toString() = String.format("(%.3f %.3f %.3f)", x, y, angNorm(h))
}

class Path(var sp: Pose, var ep: Pose){
    constructor(): this(Pose(), Pose())


}

class Point(var x: Double, var y: Double){
    constructor(): this(0.0, 0.0)
    constructor( list: List<Double>): this(list[0], list[1])

    operator fun get(i: Int) = when (i) {
        0 -> x
        else -> y
    }

    operator fun minus(point: Point): Point = Point(x - point.x, y - point.y)

    operator fun plus(point: Point): Point = Point(x + point.x, y + point.y)

    operator fun div(a: Double): Point = Point(x/a, y/a)

    fun rotate(angle: Double): Point{
        return Point(
            x * cos(angle) - y * sin(angle),
            x * sin(angle) + y * cos(angle))
    }
    @SuppressLint("DefaultLocale")
    override fun toString() = String.format("(%.3f %.3f)", x, y)

    operator fun set(i: Int, value: Double): Point {
        val pt = Point()
        pt[i] = value

        return pt
    }
}

class VecVec2D(var lld: List<List<Double>>){
    private fun toPoints(): PointVec{
        return PointVec(
            Point(lld[0][0], lld[0][1]),
            Point(lld[1][0], lld[1][1]),
            Point(lld[2][0], lld[2][1]),
            Point(lld[3][0], lld[3][1])
        )
    }

    operator fun get(i: Int): Point{
        val pv = this.toPoints()
        return when(i){
            0 -> pv[0]
            1 -> pv[1]
            2 -> pv[2]
            else -> pv[3]
        }
    }

    fun rotate(alpha: Double): PointVec{
        val pv = this.toPoints()
        return pv.rotate(alpha)
    }

    @SuppressLint("DefaultLocale")
    override fun toString() = String.format("(%.1f %.1f) (%.1f %.1f) (%.1f %.1f) (%.1f %.1f)",
        lld[0][0], lld[0][1],
        lld[1][0], lld[1][1],
        lld[2][0], lld[2][1],
        lld[3][0], lld[3][1])
}


class PointVec(val lp: List<Point>){
    constructor(p1: Point, p2: Point, p3: Point, p4: Point): this(listOf(p1, p2, p3, p4))

    operator fun get(i: Int): Point = lp[i]
    operator fun get(i: Int, j: Int): Double = lp[i][j]

    fun rotate(alpha: Double): PointVec{
        for(i in 0..3){
            lp[i][0] = lp[i][0] * cos(alpha) - lp[i][1] * sin(alpha)
            lp[i][1] = lp[i][0] * sin(alpha) + lp[i][1] * cos(alpha)
        }

        return PointVec(lp)
    }
}