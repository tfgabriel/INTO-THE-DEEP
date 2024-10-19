package org.firstinspires.ftc.teamcode.BOT_CONFIG

class robot(var isAuto: Boolean, var isRed: Boolean) {
    constructor(isAuto: Boolean): this(isAuto, true)

    fun start(){
        base_init()
        if(isAuto)
            init_auto(isRed)
        init_systems()
    }

    //dash, telemetry
    fun base_init(){

    }

    //all systems
    fun init_systems(){

    }

    //camera, localization
    fun init_auto(isRed: Boolean){

    }
}