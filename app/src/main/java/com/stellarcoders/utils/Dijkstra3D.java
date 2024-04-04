package com.stellarcoders.utils;

import gov.nasa.arc.astrobee.types.Point;

import com.stellarcoders.Area;
import com.stellarcoders.CheckPoints;
import com.stellarcoders.ConstAreas;
import com.stellarcoders.utils.PointI;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Stack;
import android.util.Log;

public class Dijkstra3D {
    private final Point[][][] points;
    private final Double INF = Double.MAX_VALUE;
    private CheckPoints meta = new CheckPoints();
    public Dijkstra3D(){
        points = new Point[meta.num_div_x][meta.num_div_y][meta.num_div_z];
    }

    private final int[] dx = {
            -1, 0, 1,
            -1, 0, 1,
            -1, 0, 1,
            -1, 0, 1,
            -1, /*0,*/ 1,
            -1, 0, 1,
            -1, 0, 1,
            -1, 0, 1,
            -1, 0, 1};
    private final int[] dy = {
            -1,-1,-1,
             0, 0, 0,
             1, 1, 1,
            -1,-1,-1,
             0, /*0,*/ 0,
             1, 1, 1,
            -1,-1,-1,
             0, 0, 0,
             1, 1, 1};

    private final int[] dz = {
            -1,-1,-1,
            -1,-1,-1,
            -1,-1,-1,
             0, 0, 0,
             0, /*0,*/ 0,
             0, 0, 0,
             1, 1, 1,
             1, 1, 1,
             1, 1, 1};

    private final double[] cost = {
            1.7,1.4,1.7,
            1.4,1,1.4,
            1.7,1.4,1.7,

            1.4, 1, 1.4,
            1, /*0,*/ 1,
            1.4, 1, 1.4,

            1.7, 1.4, 1.7,
            1.4, 1, 1.4,
            1.7, 1.4, 1.7
    };

    public ArrayList<Point> dijkstra(Point start_p, Point goal_p){
        CheckPoints checkPoints = new CheckPoints();
        PointI start = checkPoints.Point2I(start_p);
        PointI goal =  checkPoints.Point2I(goal_p);
        Log.i("StellarCoders",String.format("Dijkstra Called. Start : %s | Goal: %s",start_p,goal_p));
        // Goalが進入禁止エリアのときの処理 (BFS)
        Queue<PointI> que_bfs = new ArrayDeque<PointI>();
        double[][][] dist_bfs = new double[meta.num_div_x + 10][meta.num_div_y + 10][meta.num_div_z + 10];
        for (int i = 0; i < meta.num_div_x + 10; i++) {
            for (int j = 0; j < meta.num_div_y + 10; j++) {
                for (int k = 0; k < meta.num_div_z + 10; k++) {
                    dist_bfs[i][j][k] = INF;
                }
            }
        }

        que_bfs.add(goal);
        dist_bfs[goal.getX()][goal.getY()][goal.getZ()] = 1;
        while(!que_bfs.isEmpty()){
            PointI p = que_bfs.poll();
            if(checkPoints.checkPoints[p.getX()][p.getY()][p.getZ()]){
                goal = new PointI(p.getX(),p.getY(),p.getZ());
                break;
            }
            for (int d = 0; d < 26; d++) {
                if(Math.min(p.getX() + dx[d],Math.min(p.getY() + dy[d],p.getZ() + dz[d])) < 0)continue;
                if(p.getX() + dx[d] >= meta.num_div_x)continue;
                if(p.getY() + dy[d] >= meta.num_div_y)continue;
                if(p.getZ() + dz[d] >= meta.num_div_z)continue;
                if(dist_bfs[p.getX() + dx[d]][p.getY()+ dy[d]][p.getZ() + dz[d]] == INF) {
                    que_bfs.add(new PointI(p.getX() + dx[d], p.getY() + dy[d], p.getZ() + dz[d]));
                    dist_bfs[p.getX() + dx[d]][p.getY()+ dy[d]][p.getZ() + dz[d]] = 1;
                }
            }
        }
        // BFS処理ここまで


        Log.i("StellarCoders",String.format("Dijkstra Goal is %s",goal_p));
        PriorityQueue<Node> que = new PriorityQueue<Node>(new NodeComparator());
        double[][][] dist = new double[meta.num_div_x + 10][meta.num_div_y + 10][meta.num_div_z + 10];
        Node[][][] prev = new Node[meta.num_div_x + 10][meta.num_div_y + 10][meta.num_div_z + 10];
        for (int i = 0; i < meta.num_div_x + 10; i++) {
            for (int j = 0; j < meta.num_div_y + 10; j++) {
                for (int k = 0; k < meta.num_div_z + 10; k++) {
                    dist[i][j][k] = INF;
                    prev[i][j][k] = new Node();
                }
            }
        }
        Log.i("StellarCoders", "Dijkstra Start");
        Log.i("StellarCoders",String.format("Start Point is %d,%d,%d",start.getX(),start.getY(),start.getZ()));
        Log.i("StellarCoders",String.format("Goal Point is %d,%d,%d",goal.getX(),goal.getY(),goal.getZ()));
        dist[start.getX()][start.getY()][start.getZ()] = 0;
        Node s = new Node();
        s.p = start;

        que.add(s);
        Node trace = new Node();
        while(!que.isEmpty()){
            Node q = que.poll();
            PointI p = q.p;
            int x = p.getX();
            int y = p.getY();
            int z = p.getZ();

            //Log.i("StellarCoders",String.format("Dijkstra Logging : %s",q.toString()));
            Double distance = q.d;
            for(int d = 0;d < 26;d++){
                //border check
                if(Math.min(x + dx[d],Math.min(y + dy[d],z + dz[d])) < 0)continue;
                if(x + dx[d] >= meta.num_div_x)continue;
                if(y + dy[d] >= meta.num_div_y)continue;
                if(z + dz[d] >= meta.num_div_z)continue;
                if(!meta.checkPoints[x + dx[d]][y + dy[d]][z + dz[d]])continue;
                if(dist[x + dx[d]][y + dy[d]][z + dz[d]] > distance + cost[d]){
                    Node nxt = new Node();
                    nxt.p = new PointI(x + dx[d],y + dy[d],z + dz[d]);
                    nxt.d = distance + cost[d];
                    nxt.dir = d;
                    dist[x + dx[d]][y + dy[d]][z + dz[d]] = distance + cost[d];
                    prev[x + dx[d]][y + dy[d]][z + dz[d]] = q;
                    que.add(nxt);
                }
            }
        }


        Log.i("StellarCoders","Dijkstra finish. Start construct path");
        CheckPoints cp = new CheckPoints();
        // re-construct path
        ArrayList<Point> path = new ArrayList<>();
        trace = prev[goal.getX()][goal.getY()][goal.getZ()];
        Log.i("StellarCoders","Trace initial: " + trace.p.toString());
        while(!trace.p.isNan()){
            Log.i("StellarCoders",trace.p.toString());
            path.add(cp.idx2Point(trace.p));
            trace = prev[trace.p.getX()][trace.p.getY()][trace.p.getZ()];
        }
        Log.i("StellarCoders","Dijkstra path construct finished");
        Collections.reverse(path);
        path.add(goal_p);
        return path;
    }

}
