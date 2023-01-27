package org.firstinspires.ftc.teamcode.util;

public enum Height {
     // junction
     NONE(10),
     GROUND(-25),
     LOW(-400),
     MEDIUM(-855),
     HIGH(-1800),
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
