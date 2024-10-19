package org.firstinspires.ftc.teamcode.ALGORITHMS

class stateArray(val position1: Double, val position2: Double, val position3: Double){
    constructor(pos1: Double, pos2: Double): this(pos1, pos2, 0.0)
    constructor(pos1: Double): this(pos1, 0.0, 0.0)

    operator fun get(i: Int) = when (i) {
        0 -> position1
        1 -> position2
        else -> position3
    }
}