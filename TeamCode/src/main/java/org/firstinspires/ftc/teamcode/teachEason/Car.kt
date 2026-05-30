package org.firstinspires.ftc.teamcode.teachEason

data class Car(
    val brand: String,
    val model: String,
    val year: Int
) {
    companion object {
        val TESLA_MODEL_3 = Car("Tesla", "Model 3", 2020)
    }
}
