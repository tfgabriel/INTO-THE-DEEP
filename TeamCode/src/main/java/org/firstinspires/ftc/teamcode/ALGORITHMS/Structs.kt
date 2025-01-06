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

class Pose(@JvmField var x: Double, @JvmField var y: Double, @JvmField var h: Double, @JvmField var vel: Double){
    constructor(): this(0.0, 0.0, 0.0, 0.0)
    constructor(x: Double, y: Double): this(x, y, 0.0, 0.0)
    constructor(point: Vec2D, h: Double, vel: Double): this(point.x, point.y, h, vel)
    constructor(sparkpos: SparkFunOTOS.Pose2D): this(sparkpos.x, sparkpos.y, sparkpos.h, 0.0)
    constructor(sparkpos: SparkFunOTOS.Pose2D, vel: Double): this(sparkpos.x, sparkpos.y, angNorm( sparkpos.h), vel)

    operator fun plus(pose: Pose): Pose = Pose(x + pose.x, y + pose.y, h + pose.h, vel + pose.vel)

    operator fun minus(pose: Pose): Pose = Pose(x - pose.x, y - pose.y, h - pose.h, vel - pose.vel)

    operator fun times(a: Double): Pose = Pose(a * x, a * y, a * h, vel)


    fun rotate(angle: Double) = Pose(x * cos(angle) - y * sin(angle),
                                     x * sin(angle) + y * cos(angle),
                                        angNorm(h + angle), vel)

    fun distance(): Double = sqrt(x*x + y*y)

    fun point(): Vec2D = Vec2D(x, y)

    @SuppressLint("DefaultLocale")
    override fun toString() = String.format("(%.3f %.3f %.3f)", x, y, angNorm(h))
}

class Path(var sp: Pose, var ep: Pose){
    constructor(): this(Pose(), Pose())
}

class Trajectory(vararg path: Path){
    var paths = path.asList()

    operator fun get(i: Int) = paths[i]

    val size: Int
        get(){
            return paths.size
        }
}

class Vec2D(var x: Double, var y: Double){
    constructor(): this(0.0, 0.0)
    constructor( list: List<Double>): this(list[0], list[1])

    operator fun get(i: Int) = when (i) {
        0 -> x
        else -> y
    }

    operator fun minus(point: Vec2D): Vec2D = Vec2D(x - point.x, y - point.y)

    operator fun plus(point: Vec2D): Vec2D = Vec2D(x + point.x, y + point.y)

    operator fun div(a: Double): Vec2D = Vec2D(x/a, y/a)

    operator fun times(a: Double): Vec2D = Vec2D(x*a, y*a)

    fun distance(): Double = x*x + y*y

    fun rotate(angle: Double): Vec2D {
        return Vec2D(
            x * cos(angle) - y * sin(angle),
            x * sin(angle) + y * cos(angle))
    }
    @SuppressLint("DefaultLocale")
    override fun toString() = String.format("(%.3f %.3f)", x, y)

    operator fun set(i: Int, value: Double): Vec2D {
        val pt = Vec2D()
        pt[i] = value

        return pt
    }
}

class VecVec2D(var lld: List<List<Double>>){
    private fun toPoints(): PointVec {
        return PointVec(
            Vec2D(lld[0][0], lld[0][1]),
            Vec2D(lld[1][0], lld[1][1]),
            Vec2D(lld[2][0], lld[2][1]),
            Vec2D(lld[3][0], lld[3][1])
        )
    }

    operator fun get(i: Int): Vec2D {
        val pv = this.toPoints()
        return when(i){
            0 -> pv[0]
            1 -> pv[1]
            2 -> pv[2]
            else -> pv[3]
        }
    }

    fun rotate(alpha: Double): PointVec {
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


class PointVec(val lp: List<Vec2D>){
    constructor(p1: Vec2D, p2: Vec2D, p3: Vec2D, p4: Vec2D): this(listOf(p1, p2, p3, p4))

    operator fun get(i: Int): Vec2D = lp[i]
    operator fun get(i: Int, j: Int): Double = lp[i][j]

    fun rotate(alpha: Double): PointVec {
        for(i in 0..3){
            lp[i][0] = lp[i][0] * cos(alpha) - lp[i][1] * sin(alpha)
            lp[i][1] = lp[i][0] * sin(alpha) + lp[i][1] * cos(alpha)
        }

        return PointVec(lp)
    }
}


//ITS AN INTERSECTION
//BETWEEN A CIRCLE AND A LINE
//NOT A LINE SEGMENT
//YOU MUST CHECK IF THE SOLUTIONS ARE IN BOUNDS

class Intersection(val trajectory: Trajectory, val center: Pose, val radius: Double, val target_path_index: Int){
    constructor(path: Path, center: Pose, radius: Double): this(Trajectory(path), center, radius, 0)
    constructor(trajectory: Trajectory, center: Pose, radius: Double): this(trajectory, center, radius, 0)


    var A: Double = 0.0
    var B: Double = 0.0
    var C: Double = 0.0

    private var x_s = -B / (2 * A)
    private var y_s = slope() * x_s + constant()

    private var x_1 = -(B - sqrt(discriminant())) / (2 * A)
    private var y_1 = slope()*x_1 + constant()

    private var x_2 = -(B + sqrt(discriminant())) / (2 * A)
    private var y_2 = slope() * x_2 + constant()

    var single_solution: Vec2D = Vec2D(x_s, y_s)
    var first_solution: Vec2D = Vec2D(x_1, y_1)
    var second_solution: Vec2D = Vec2D(x_2, y_2)

    var exception: Vec2D = Vec2D(-1000.0, -1000.0)
    //to check if a value is between two bounds, you normally do it like this a < x < b
    //in this scenario, we don't know whether a is bigger than b, so we switch them around if they arent to make the same comparison regardless of that
    //because the robots "positive x" is actually negative, the math is switched, usually the ep should be bigger than the sp, but here the general case is the other way round

    private var start_x_bound =
        if(trajectory[target_path_index].ep.x >= trajectory[target_path_index].sp.x){
            trajectory[target_path_index].ep.x
        }
        else{
            trajectory[target_path_index].sp.x
        }

    private var end_x_bound =
        if(trajectory[target_path_index].ep.x >= trajectory[target_path_index].sp.x){
            trajectory[target_path_index].sp.x
        }
        else{
            trajectory[target_path_index].ep.x
        }


    private fun isInBounds(point: Vec2D) =
        if(point.x in start_x_bound..end_x_bound)
            point
        else
            exception

    fun only_solution(): Vec2D {
        return isInBounds(single_solution)
    }

    fun closest_solution(): Vec2D {
        val end_point = trajectory[target_path_index].ep.point()
        val first_distance = (first_solution-end_point).distance()
        val second_distance = (second_solution-end_point).distance()

        return if(first_distance <= second_distance)
            isInBounds(first_solution)
        else
            isInBounds(second_solution)
    }

    private fun slope(): Double{
        val diff = trajectory[target_path_index].ep - trajectory[target_path_index].sp
        return diff.y / diff.x
    }

    private fun constant(): Double{
        return -slope()*trajectory[target_path_index].sp.x+trajectory[target_path_index].sp.y
    }

    fun discriminant(): Double{
        A = 1 + slope()* slope()
        B = 2*(slope()*constant()-slope()*center.y-center.x)
        C = center.y*center.y - radius*radius + center.x*center.x - 2*constant()*center.y + constant()*constant()

        return B*B - 4*A*C
    }
}