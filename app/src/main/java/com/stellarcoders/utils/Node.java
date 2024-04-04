package com.stellarcoders.utils;

import java.util.Comparator;

public class Node {
    public PointI p;
    public Double d;
    public Integer dir;

    Node() {
        p = new PointI();
        d = (double) 0;
        dir = 0;
    }

    public String toString(){
        return this.p.toString() + " / Distance :" + d.toString() ;
    }
}
class NodeComparator implements Comparator<Node> {
    @Override
    public int compare(Node obj1, Node obj2) {
        double d1 = obj1.d;
        double d2 = obj2.d;

        //昇順
        return Double.compare(d1, d2);
    }
}