package com.stellarcoders;

import java.util.ArrayList;

import gov.nasa.arc.astrobee.types.Point;

public class ConstPoints {
    public final Point start = new Point(9.815, -9.806, 4.293);
    public final Point goal = new Point(11.143, -6.7607, 4.9654); //Astronaut


    public final ArrayList<Point> points = new ArrayList<Point>() {
        {
            // 平面情報が与えられている。ひとまず中心をポイントとして扱う
            add(new Point(10.95, -9.92284, 5.195)); //Area1　(10.95,いいかんじ,5.195)
            add(new Point (10.925,-8.875,4.48 + 0.0)); //Area2  (10.925,-8.875,4.48 + 0.0) // zがkiz外
            add(new Point(10.925, -7.925, 4.48 + 0.0)); //Area3
            add(new Point(10.51, -6.8525, 4.945)); // Area4
        }
    };

    // 素のターゲット座標。このまま移動命令を出すと壁にめり込む
    public final ArrayList<Point> targets = new ArrayList<Point>() {
        {
            // 1.03 x 0 x 0.75
            add(new Point((10.42 + 11.48) / 2, (-10.58 - 10.58) / 2, (4.82 + 5.57) / 2)); //Area1
            add(new Point((10.3 + 11.55) / 2, (-9.25 - 8.5) / 2, (3.76203 + 3.76203) / 2)); //Area2
            add(new Point((10.3 + 11.55) / 2, (-8.4 - -7.45) / 2, (3.76093 + 3.76093) / 2)); //Area3
            add(new Point((9.866984 + 9.866984) / 2, (-7.34 - 6.365) / 2, (4.32 + 5.57) / 2)); // Area4
        }
    };

}
