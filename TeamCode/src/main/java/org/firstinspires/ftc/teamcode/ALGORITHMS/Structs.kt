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

class Point(var x: Double, var y: Double){
    constructor(): this(0.0, 0.0)

    operator fun get(i: Int) = when (i) {
        0 -> x
        else -> y
    }

    operator fun minus(point: Point): Point = Point(x - point.x, y - point.y)

    operator fun plus(point: Point): Point = Point(x + point.x, y + point.y)

}

class PointVec(var p1: Point, var p2: Point, var p3: Point, var p4: Point){
    operator fun get(i: Int) = when (i) {
        0 -> p1
        1 -> p2
        2 -> p3
        else -> p4
    }
}