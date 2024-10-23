package org.firstinspires.ftc.teamcode.ALGORITHMS

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

    operator fun plus(pose: Pose): Pose = Pose(x + pose.x, y + pose.y, h + pose.h)

    operator fun minus(pose: Pose): Pose = Pose(x - pose.x, y - pose.y, h - pose.h)

    operator fun times(a: Double): Pose = Pose(a * x, a * y, a * h)

    fun distance(): Double = sqrt(x*x - y*y)
}