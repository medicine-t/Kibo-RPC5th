package com.stellarcoders.utils;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

public class Vector3 {
    private double x;
    private double y;
    private double z;


    public Vector3(double x, double y, double z){
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public Vector3(Point p1, Point p2){
        this.x = p1.getX() - p2.getX();
        this.y = p1.getY() - p2.getY();
        this.z = p1.getZ() - p2.getZ();
    }

    public double getX(){
        return this.x;
    }

    public double getY() {
        return this.y;
    }

    public double getZ() {
        return this.z;
    }

    public Vector3 subtract(Vector3 v){
        return new Vector3(this.x - v.x,this.y - v.y,this.z - v.y);
    }

    public Vector3 add(Vector3 v){
        return new Vector3(this.x + v.x, this.y + v.y, this.z + v.z);
    }

    public Vector3 prod(double d){
        return new Vector3(d * this.x,d * this.y,d * this.z);
    }


    public double dot(Vector3 v){
        return this.x * v.x + this.y * v.y + this.z * v.z;
    }

    public Vector3 cross(Vector3 v){
        return new Vector3(this.y * v.z - this.z * v.y,this.z * v.x - this.x * v.z,this.x * v.y - this.y * v.x);
    }

    public double norm(){
        return Math.sqrt(x*x + y*y + z*z);
    }

    public Vector3 normalize(){
        return new Vector3(x / norm(), y/ norm(), z/norm());
    }

    public Vector3 rotate(Quaternion q){
        Quaternion p = new Quaternion((float) this.x,(float) this.y,(float) this.z,0);
        Quaternion rotated = Utils.quaternionProd(Utils.quaternionProd(q,p),Utils.inverseQuaternion(q));
        return new Vector3(rotated.getX(),rotated.getY(),rotated.getZ());
    }

    /**
     * x,y,z -> way[0], way[1], way[2]の順に並べる
     *
     * 例
     * 2,1,0なら xyz -> zyx
     * 0,2,1なら　xyz -. xzy
     *
     * 0: xyz
     * 1: xzy
     * 2: yxz
     * 3: yzx
     * 4: zxy
     * 5: zyx
     *
     * @param way
     * @return
     */
    public Vector3 transpose(int[] way){
        if(way[0] == 0 && way[1] == 1 && way[2] == 2){
            return new Vector3(this.x,this.y,this.z);
        } else if(way[0] == 0 && way[1] == 2 && way[2] == 1){
            return new Vector3(this.x,this.z,this.y);
        } else if(way[0] == 1 && way[1] == 0 && way[2] == 2){
            return new Vector3(this.y,this.x,this.z);
        } else if(way[0] == 1 && way[1] == 2 && way[2] == 0){
            return new Vector3(this.y,this.z,this.x);
        } else if(way[0] == 2 && way[1] == 0 && way[2] == 1){
            return new Vector3(this.z,this.x,this.y);
        } else if(way[0] == 2 && way[1] == 1 && way[2] == 0){
            return new Vector3(this.z,this.y,this.x);
        }
        else return new Vector3(this.x,this.y,this.z);
    }


}
