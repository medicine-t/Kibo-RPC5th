package com.stellarcoders;

import java.util.ArrayList;

import gov.nasa.arc.astrobee.types.Quaternion;

public class ConstQuaternions {
    public final Quaternion start = new Quaternion(1, 0, 0, 0);
    public final Quaternion goal = new Quaternion(0f, 0f, 0.707f, 0.707f); //Astronaut

    public final ArrayList<Quaternion> points = new ArrayList<Quaternion>() {
        {
//                add(new Quaternion(0, 0 ,-0.707f, 0.707f));
            add(new Quaternion(0, 0, -0.707f, 0.707f));
            add(new Quaternion(0.5f,0.5f,-0.5f,0.5f));
            add(new Quaternion(0.5f,0.5f,-0.5f,0.5f));
            add(new Quaternion(0f, 0f, -1f, 0f));
        }
    };

    public final ArrayList<Quaternion> targets = new ArrayList<Quaternion>() {
        {
            add(new Quaternion(0, 0, -0.707f, 0.707f));
            add(new Quaternion(0.5f,0.5f,-0.5f,0.5f));
            add(new Quaternion(0.5f,0.5f,-0.5f,0.5f));
            add(new Quaternion(0f, 0f, -1f, 0f));
        }
    };

}
