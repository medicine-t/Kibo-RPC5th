package com.stellarcoders.utils;

public class PointI {
    private final Integer x;
    private final Integer y;
    private final Integer z;

    public PointI(Integer x, Integer y, Integer z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    PointI(){
        this.x = this.y = this.z = -1;
    }

    public boolean isNan(){
        return this.x == -1 || this.y == -1 || this.z == -1;
    }

    public Integer getX(){
        return this.x;
    }

    public Integer getY(){
        return this.y;
    }

    public Integer getZ() {
        return this.z;
    }


    public String toString(){
        return "[PointI] X: " + this.x.toString() + " Y: " + this.y.toString() + " Z: " + this.z.toString();
    }
}
