package com.stellarcoders;
import java.util.ArrayList;

import gov.nasa.arc.astrobee.types.Point;
public class Area {
    public final float x_min;
    public final float y_min;
    public final float z_min;

    public final float x_max;
    public final float y_max;
    public final float z_max;

    Area(float x_min,float y_min,float z_min,float x_max,float y_max, float z_max){
        this.x_min = x_min;
        this.y_min = y_min;
        this.z_min = z_min;
        this.x_max = x_max;
        this.y_max = y_max;
        this.z_max = z_max;
    }

    public final Boolean isInclude(Point point){
        if(x_min <= point.getX() && point.getX() <= x_max){
            if(y_min <= point.getY() && point.getY() <= y_max){
                if(z_min <= point.getZ() && point.getZ() <= z_max){
                    return true;
                }
            }
        }
        return false;
    }

    public final Boolean isIntersect(Area a){
        return ((a.x_min <= this.x_max && a.x_max >= this.x_min) &&
                (a.y_min <= this.y_max && a.y_max >= this.y_min) &&
                (a.z_min <= this.z_max && a.z_max >= this.z_min));
    }

    public final Boolean isInclude(Area a){
        Area b = this;
        int sum = 0;
        sum += ((a.x_min <= b.x_min && b.x_max <= a.x_max) || (b.x_min <= a.x_min && a.x_max <= b.x_max)) ? 1 : 0;
        sum += ((a.y_min <= b.y_min && b.y_max <= a.y_max) || (b.y_min <= a.y_min && a.y_max <= b.y_max)) ? 1 : 0;
        sum += ((a.z_min <= b.z_min && b.z_min <= a.z_max) || (b.z_min <= a.z_min && a.z_min <= b.z_max)) ? 1 : 0;
        return sum >= 2;
    }

    public final Area mergeArea(Area a){
        return new Area(Math.min(a.x_min,this.x_min),Math.min(a.y_min,this.y_min),Math.min(a.z_min,this.z_min),Math.max(a.x_max,this.x_max),Math.max(a.y_max,this.y_max),Math.max(a.z_max,this.z_max));
    }

    public ArrayList<Point[]> getPolys(){
        ArrayList<Point[]> polys = new ArrayList<>();
        // z軸並行
        polys.add(new Point[]{new Point(x_max,y_max,z_max), new Point(x_min,y_max,z_max), new Point(x_min,y_min,z_max)});
        polys.add(new Point[]{new Point(x_max,y_max,z_max), new Point(x_max,y_min,z_max), new Point(x_min,y_min,z_max)});

        polys.add(new Point[]{new Point(x_max,y_max,z_min), new Point(x_min,y_min,z_min), new Point(x_min,y_max,z_min)});
        polys.add(new Point[]{new Point(x_max,y_max,z_min), new Point(x_max,y_min,z_min), new Point(x_min,y_min,z_min)});

        // y軸並行
        polys.add(new Point[]{new Point(x_min,y_max,z_min), new Point(x_max,y_max,z_min), new Point(x_max,y_max,z_max)});
        polys.add(new Point[]{new Point(x_min,y_max,z_min), new Point(x_min,y_max,z_max), new Point(x_max,y_max,z_max)});

        polys.add(new Point[]{new Point(x_min,y_min,z_min), new Point(x_max,y_min,z_min), new Point(x_max,y_min,z_max)});
        polys.add(new Point[]{new Point(x_min,y_min,z_min), new Point(x_min,y_min,z_max), new Point(x_max,y_min,z_max)});

        // x軸並行
        polys.add(new Point[]{new Point(x_max,y_min,z_min), new Point(x_max,y_max,z_min), new Point(x_max,y_max,z_max)});
        polys.add(new Point[]{new Point(x_max,y_min,z_min), new Point(x_max,y_min,z_max), new Point(x_max,y_max,z_max)});

        polys.add(new Point[]{new Point(x_min,y_min,z_min), new Point(x_min,y_max,z_min), new Point(x_min,y_max,z_max)});
        polys.add(new Point[]{new Point(x_min,y_min,z_min), new Point(x_min,y_min,z_max), new Point(x_min,y_max,z_max)});
        return polys;
    }
}
