package org.firstinspires.ftc.teamcode.library.game;

public enum Ball {
    NECTAR(new Nectar()),
    POLLEN(new Pollen());
    public final GameElement e;
    Ball(GameElement gameElement) {
        this.e = gameElement;
    }
}
