package com.stellarcoders.utils;

import android.util.Log;

import com.stellarcoders.CheckPoints;
import com.stellarcoders.ConstQuaternions;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Vector;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcApi;

public class Utils {
    public static Quaternion inverseQuaternion(Quaternion q){
        return new Quaternion(q.getX() * -1,q.getY() * -1, q.getZ() * -1,q.getW());
    }

    public static Double distance3DSquare(Point p1,Point p2){
        return (p1.getX() - p2.getX()) * (p1.getX() - p2.getX()) +  (p1.getY() - p2.getY()) * (p1.getY() - p2.getY()) +(p1.getZ() - p2.getZ()) * (p1.getZ() - p2.getZ());
    }

    /**
     *
     * @param p1    始点
     * @param p2    終点
     * @param a   衝突判定の面の頂点[3]
     * @return 衝突時 true
     *
     * 参考: https://pheema.hatenablog.jp/entry/ray-triangle-intersection
     */
    public static Boolean isPolyLineCollision(Point p1,Point p2,Point[] a){
        Point a0 = a[0];
        Point a1 = a[1];
        Point a2 = a[2];

        Vector3 d = new Vector3(p2,p1).normalize();
        Vector3 e1 = new Vector3(a1,a0);
        Vector3 e2 = new Vector3(a2,a0);
        Vector3 r = new Vector3(p1,a0);

        final double eps = 1e-6;
        Vector3 alpha = d.cross(e2);
        double det = e1.dot(alpha);
        // 三角形に対して、レイが平行に入射するような場合 det = 0 となる。
        // det が小さすぎると 1/det が大きくなりすぎて数値的に不安定になるので
        // det ≈ 0 の場合は交差しないこととする。
        if (Math.abs(det) < eps) {
            return false;
        }

        double invDet = 1.0 / det;
        double u = alpha.dot(r) * invDet;
        if (u < 0.0f || u > 1.0f) {
            return false;
        }

        Vector3 beta = r.cross(e1);
        double v = d.dot(beta) * invDet;
        if (v < 0.0f || u + v > 1.0f) {
            return false;
        }

        double t = e2.dot(beta) * invDet;
        if (t <= 0.0f) {
            return false;
        }

        if (Double.isNaN(t)){
            //Log.e("StellarCoders",String.format("NaN appeared. det : %e",det));
            return false;
        }
        double targetNorm = new Vector3(p2,p1).norm();
        boolean retValue = !(t - targetNorm > eps);
        //Log.i("StellarCoders",String.format("Collision t:%.3f, u:%.3f, v:%.3f",t,u,v));
        //Log.i("StellarCoders",String.format("Start %s, Destination: %s. norm is %.3f, t - targetNorm %.3f, return value: %b",Utils.Point2str(p1),Utils.Point2str(p2),targetNorm,t - targetNorm,retValue));

        return retValue;
    }

    public static Mat calibratedNavCam(KiboRpcApi api){
        double[][] camStatistics = api.getNavCamIntrinsics();
        double[] camMtx = camStatistics[0];
        double[] dist = camStatistics[1];
        Mat navCam = api.getMatNavCam();
        Mat camMtxMat = new Mat(3,3, CvType.CV_64FC1);
        camMtxMat.put(0,0,camMtx);
        Mat distMat = new Mat(1,5,CvType.CV_64FC1);
        distMat.put(0,0,dist);

        Mat undistorted = new Mat();
        Calib3d.undistort(navCam,undistorted,camMtxMat,distMat);
        return undistorted;
    }


    public static ArrayList<Integer> searchMarker(Mat img){
        ArrayList<Integer> ret = new ArrayList<>();
        List<Mat> corners = new ArrayList<>();
        Mat markerIds = new Mat();
        Dictionary arucoDict = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        Aruco.detectMarkers(img,arucoDict,corners,markerIds);
        for (int i = 0; i < markerIds.size(0); i++) {
            ret.add((int) markerIds.get(i,0)[0]);
        }
        return ret;
    }

    public static Mat drawMarker(KiboRpcApi api, Mat img){
        ArrayList<Integer> ret = new ArrayList<>();
        List<Mat> corners = new ArrayList<>();
        Mat markerIds = new Mat();
        Dictionary arucoDict = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        Aruco.detectMarkers(img,arucoDict,corners,markerIds);
        Aruco.drawDetectedMarkers(img,corners);
        return img;
    }

    public static Mat drawMarkerPoseEstimation(KiboRpcApi api){
        double[][] camStatistics = api.getNavCamIntrinsics();
        double[] camMtx = camStatistics[0];
        double[] dist = camStatistics[1];
        Mat camMtxMat = new Mat(3,3, CvType.CV_64FC1);
        camMtxMat.put(0,0,camMtx);
        Mat distMat = new Mat(1,5,CvType.CV_64FC1);
        distMat.put(0,0,dist);

        Mat navCam = api.getMatNavCam();
        ArrayList<Integer> ret = new ArrayList<>();
        List<Mat> corners = new ArrayList<>();
        Mat markerIds = new Mat();
        Dictionary arucoDict = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        Aruco.detectMarkers(navCam,arucoDict,corners,markerIds);

        Mat rvecs = new Mat();
        Mat tvecs = new Mat();
        Mat _objPoints = new Mat();

        Aruco.estimatePoseSingleMarkers(corners,0.05f,camMtxMat,distMat,rvecs,tvecs,_objPoints);
        for (int i = 0; i < rvecs.height(); i++) {
            Aruco.drawAxis(navCam,camMtxMat,distMat,rvecs.row(i),tvecs.row(i),0.1f);
        }
        return navCam;
    }

    public static Vector3 cam2center = new Vector3(0 * 0.1177, -0.0422, -0.0826).prod(-1);
    public static Vector3 center2laser = new Vector3(0 * 0.1302, 0.0572, -0.1111);
    //public static Vector3 cam2laser = Utils.cam2center.add(Utils.center2laser); // 0,0.0994, -0.0285
    public static Vector3 cam2laser = new Vector3(0,0.0994, -0.0285);
    public static Point applyPoint(Point p,Vector3 v){
        return new Point(p.getX() + v.getX(),p.getY() + v.getY(), p.getZ() + v.getZ());
    }

    public static Vector3 getDiffFromCam(KiboRpcApi api,int targetId){
        double[][] biasMarker = {
                {-0.1,+0.0375,0},
                {0.1,+0.0375,0},
                {0.1,-0.0375,0},
                {-0.1,-0.0375,0},
        };

        ConstQuaternions cq = new ConstQuaternions();
        double[][] camStatistics = api.getNavCamIntrinsics();
        double[] camMtx = camStatistics[0];
        double[] dist = camStatistics[1];
        Mat camMtxMat = new Mat(3,3, CvType.CV_64FC1);
        camMtxMat.put(0,0,camMtx);
        Mat distMat = new Mat(1,5,CvType.CV_64FC1);
        distMat.put(0,0,dist);

        Mat navCam = api.getMatNavCam();
        ArrayList<Integer> ret = new ArrayList<>();
        List<Mat> corners = new ArrayList<>();
        Mat markerIds = new Mat();
        Dictionary arucoDict = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        Aruco.detectMarkers(navCam,arucoDict,corners,markerIds);

        Mat rvecs = new Mat();
        Mat tvecs = new Mat();
        Mat _objPoints = new Mat();

        Aruco.estimatePoseSingleMarkers(corners,0.05f,camMtxMat,distMat,rvecs,tvecs,_objPoints);
        if(rvecs.empty()){
            return new Vector3(0,0,0);
        }

        ArrayList<Integer> ids = new ArrayList<>();
        for (int i = 0; i < markerIds.size(0); i++) {
            ids.add((int) markerIds.get(i,0)[0]);
        }
        Log.i("StellarCoders",String.format("Detected : %s",ids));
        if(ids.size() == 0) return new Vector3(0,0,0);

        Vector3 target = new Vector3(0,0,0);
        for (int i = 0; i < Math.min(ids.size(),rvecs.height()); i++) {
            if((ids.get(i) - 1)/ 4 != targetId)continue;
            double[] relationalTarget = new double[]{0,0,0};
            relationalTarget = tvecs.get(i,0);
            //TODO: ここの座標がシミュレーターから推察できる結果とズレているので調査
            Log.i("StellarCoders",String.format("UnBiased %s", Arrays.toString(relationalTarget)));
            relationalTarget[0] += biasMarker[(ids.get(i) - 1 ) % 4][0];
            relationalTarget[1] += biasMarker[(ids.get(i) - 1 ) % 4][1];
            relationalTarget[2] += biasMarker[(ids.get(i) - 1 ) % 4][2];
            Log.i("StellarCoders",String.format("Biased %s", Arrays.toString(relationalTarget)));
            target = target.add(new Vector3(relationalTarget[0] - cam2laser.getY(),(relationalTarget[1] - cam2laser.getZ()),0));
        }
        Vector3 v = target.prod(1.0 / rvecs.height());
        Log.i("StellarCoders",String.format("Average diffs : %.3f %.3f %.3f",v.getX(),v.getY(),v.getZ()));

        double nx = v.getX();
        double ny = v.getY();
        double nz = v.getZ();
        double minimum_move = 0.1;
        //TODO:  最小移動距離になるようにz方向の移動距離を操作するという案
        if(nx * nx + ny * ny <= 0.03 * 0.03){
            return new Vector3(0,0,0);
        }
        //if(0.01 * 0.01 <= nx * nx + ny * ny && nx * nx + ny * ny <= minimum_move * minimum_move){
           //nz = Math.sqrt(Math.max(0,0.05 * 0.05 - nx * nx - ny * ny));
        //}
        Vector3 camPositionFixed = new Vector3(nx,ny,nz);
        //camPositionFixed = camPositionFixed.add(Utils.center2laser);
        Log.i("StellarCoders",String.format("Un-Rotated Coordinate : %.3f %.3f %.3f",camPositionFixed.getX(),camPositionFixed.getY(),camPositionFixed.getZ()));
        //Quaternion q = Utils.inverseQuaternion(api.getRobotKinematics().getOrientation());
        //Log.i("StellarCoders",String.format("Rotate Quaternion : %.3fi + %.3fj + %.3fk + %.3f",q.getX(),q.getY(),q.getZ(),q.getW()));
        Vector3 rotated_v = Utils.target2transpose((ids.get(0) - 1) / 4,camPositionFixed);
        return rotated_v;
    }

    /**
     * qp
     * @param p
     * @param q
     * @return
     * https://qiita.com/drken/items/0639cf34cce14e8d58a5#1-4-%E3%82%AF%E3%82%A9%E3%83%BC%E3%82%BF%E3%83%8B%E3%82%AA%E3%83%B3%E3%81%AE%E3%81%8B%E3%81%91%E7%AE%97
     */
    public static Quaternion quaternionProd(Quaternion q,Quaternion p){
        return new Quaternion(
                q.getW() * p.getX() - q.getZ() * p.getY() + q.getY() * p.getZ() + q.getX() * p.getW(),
                q.getZ() * p.getX() + q.getW() * p.getY() - q.getX() * p.getZ() + q.getY() * p.getW(),
                -1 * q.getY() * p.getX() + q.getX() * p.getY() + q.getW() * p.getZ() + q.getZ() * p.getW(),
                -1 * q.getX() * p.getX() - q.getY() * p.getY() - q.getZ() * p.getZ() + q.getW() * p.getW()
                );
    }

    public static String Point2str(Point p){
        return String.format("Point [%.3f, %.3f, %.3f]", p.getX(),p.getY(),p.getZ());
    }

    // return millisecond
    public static double calcAchieveTime(double distance){
        if (distance < 2.5){ // 最高速まで行かずに減速を開始するとき
            // v-t図書いて面積Sを求めてS = distanceとしてtを求めるといい
            return 2 * Math.sqrt(10 * distance) * 1000;
        }else {
            return ((distance - 2.5) * 2 + 10) * 1000 ;
        }
    }

    public static double calcMovingTime(ArrayList<Point> move_oder){
        double ret = 0.0;
        for (int i = 0; i < move_oder.size() - 1; i++) {
            ret += Utils.calcAchieveTime(Math.sqrt(Utils.distance3DSquare(move_oder.get(i),move_oder.get(i + 1))));
        }
        return ret;
    }

    /**
     *      * 0: xyz
     *      * 1: xzy
     *      * 2: yxz
     *      * 3: yzx
     *      * 4: zxy
     *      * 5: zyx
     * @param target
     * @return
     */
    public static Vector3 target2transpose(int target,Vector3 v){
        if(target == 0){
            //local  xyz -> global xzy
            //global xyz -> local xzy
            return new Vector3(v.getX(),v.getZ(),v.getY());
        }else if(target == 1){
            //local xyz -> global x -y -z
            //global x, -y, -z;
            return new Vector3(v.getX(),-v.getY(),-v.getZ());
        }else if(target == 2){
            // local xyz -> global yxz
            // global y, x, z
            return new Vector3(v.getY(),v.getX(),v.getZ());
        }else if(target == 3){
            //local xyz -> global -yz-x
            //global xyz -> -z,-x,y
            return new Vector3(-v.getZ(),-v.getX(),v.getY());
        }else if(target == 4){// xyz
            return new Vector3(v.getX(),v.getY(),v.getZ());
        }else if(target == 5){
            //local xyz -> yzx
            //global z,x,y
            return new Vector3(v.getZ(),v.getX(),v.getY());
        }
        return new Vector3(0,0,0);
    }

    public static Mat rotateImg(Mat img){
        org.opencv.core.Point center = new org.opencv.core.Point(img.cols() / 2, img.rows() / 2);
        //int center = (width/2), int(height/2))
        //回転角を指定
        double  angle = 90.0;
        //スケールを指定
        double scale = 1.0;
        //getRotationMatrix2D関数を使用
        Mat rotMat = Imgproc.getRotationMatrix2D(center,angle,scale);
        Mat warpRotateDst = new Mat();
        Imgproc.warpAffine( img, warpRotateDst, rotMat, img.size() );
        return warpRotateDst;
    }

    public static Quaternion QuatSlerp(Quaternion q1,Quaternion q2,double d){
        double qr = q1.getW() * q2.getW() + q1.getX() * q2.getX() + q1.getY() * q2.getY()
                + q1.getZ() * q2.getZ();
        double ss = 1.0 - (qr * qr);
        if (ss == 0.0){
            return new Quaternion(
                     q1.getX(),
                    q1.getY(),
                    q1.getZ(),
                    q1.getW()
            );
        }else{
            double ph = Math.acos(qr);
            if(qr < 0.0 && ph > Math.PI / 2.0){
                float s1,s2;
                qr = - q1.getW() * q2.getW() - q1.getX() * q2.getX() - q1.getY() * q2.getY()
                        - q1.getZ() * q2.getZ();
                ph = Math.acos(qr);
                s1 = (float) (Math.sin(ph * (1.0 - d)) / Math.sin(ph));
                s2 = (float) (Math.sin(ph * d) / Math.sin(ph));

                return new Quaternion(
                         q1.getX() * s1 - q2.getX() * s2,
                        q1.getY() * s1 - q2.getY() * s2,
                        q1.getZ() * s1 - q2.getZ() * s2,
                        q1.getW() * s1 - q2.getW() * s2);
            }else{
                float s1,s2;
                s1 = (float) (Math.sin(ph * (1.0 - d)) / Math.sin(ph));
                s2 = (float) (Math.sin(ph *  d       ) / Math.sin(ph));
                return new Quaternion(
                         q1.getX() * s1 + q2.getX() * s2,
                        q1.getY() * s1 + q2.getY() * s2,
                         q1.getZ() * s1 + q2.getZ() * s2,
                         q1.getW() * s1 + q2.getW() * s2);
            }
        }
    }
}
