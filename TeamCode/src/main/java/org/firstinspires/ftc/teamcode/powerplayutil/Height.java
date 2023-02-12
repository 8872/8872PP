package org.firstinspires.ftc.teamcode.powerplayutil;

public enum Height {
    // junction
    NONE(10),
    GROUND(-25),
    LOW(-300),
    MEDIUM(-755),
    HIGH(-1685),
    // cone stack
    FIRST(-142),
    SECOND(-121),
    THIRD(-87),
    FOURTH(-60);

    // height in encoder ticks
    private final int height;

    Height(int height) {
        this.height = height;
    }

    public int getHeight() {
        return height;
    }
}