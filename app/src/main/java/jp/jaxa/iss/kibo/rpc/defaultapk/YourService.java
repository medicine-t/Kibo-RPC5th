package jp.jaxa.iss.kibo.rpc.defaultapk;

import com.stellarcoders.Area;
import com.stellarcoders.CheckPoints;
import com.stellarcoders.ConstAreas;
import com.stellarcoders.ConstPoints;
import com.stellarcoders.ConstQuaternions;
import com.stellarcoders.ImageProcessor;
import com.stellarcoders.utils.Dijkstra3D;
import com.stellarcoders.utils.Utils;

import android.annotation.SuppressLint;
import android.util.Log;

import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import gov.nasa.arc.astrobee.Kinematics;
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {
    @SuppressLint("DefaultLocale")
    @Override
    protected void runPlan1() {
        GlobalContext.onCreateApplication(this.getApplicationContext());
        // write your plan 1 here
        api.startMission();

        ConstPoints pointData = new ConstPoints();
        ConstQuaternions quaternions = new ConstQuaternions();
        Thread thread = new Thread(() -> {
            //TODO: 移動順の最適化
            for (int i = 0; i < 4; i++) {
                int index = i;
                moveDijkstra(pointData.points.get(index), quaternions.points.get(index));
            }
            // データセット集める用
            moveDijkstra(pointData.goal, quaternions.goal);
            /* *********************************************************************** */
            /* Write your code to recognize type and number of items in the each area! */
            /* *********************************************************************** */

            // When you move to the front of the astronaut, report the rounding completion.
            api.reportRoundingCompletion();
            try {
                Thread.sleep(1000 * 60);
            } catch (InterruptedException e) {
                Log.w("StellarCoders",e);
            }

            //        moveDijkstra(pointData.goal, quaternions.goal);

        });
        thread.start();

        // Get a camera image.
        int cnt = 0;
        ImageProcessor imageProcessor = new ImageProcessor();
        HashMap<Integer,String> itemMapping = new HashMap<>();
        HashMap<String,Integer> keyMapping = new HashMap<>();
        boolean alive = true;
        while (thread.isAlive()) {
//            api.saveMatImage(Utils.calibratedNavCam(api), String.format("Image%d.png",cnt));
            List<Mat> fields = imageProcessor.extractTargetField(api);
            for (Mat field : fields){
                api.saveMatImage(field, String.format("ExtractImage_%d.png",cnt));
                HashMap<String, Integer> result = imageProcessor.detectItems(field);
                List<Integer> ids = Utils.searchMarker(field);
                int labelCount = result.keySet().size();
                if (labelCount == 1 && ids.size() == 1){
                    for(Map.Entry<String,Integer> entry : result.entrySet()) {
                        itemMapping.put(ids.get(0) - 100, entry.getKey());
                        if(ids.get(0) - 100 != 0) {
                            keyMapping.put(entry.getKey(),ids.get(0) - 100);
                            api.setAreaInfo(ids.get(0) - 100, entry.getKey(), entry.getValue());
                        } else {
                            api.notifyRecognitionItem();
                            thread.interrupt();
                            alive = false;
                        }

                    }
                }
            }
            if (!alive)break;
            cnt++;
        }

//        // determine what keys is labeled
//        for(Map.Entry<Integer, String> entry : itemMapping.entrySet()) {
//            // 最頻値を出す
//            ArrayList<Integer> result = areaInfo.get(entry.getKey());
//            HashMap<Integer,Integer> tmp = new HashMap<>();
//            for (Integer ret:result ) {
//                tmp.put(tmp.getOrDefault(ret, 0) + 1);
//            }
//
//        }

        Log.i("StellarCoders","move to goal");
        api.saveMatImage(Utils.calibratedNavCam(api), String.format("Image%d.png",cnt));
        Log.i("StellarCoders",String.format("%s,%s",keyMapping,itemMapping));
        if(itemMapping.get(0) != null || keyMapping.get(itemMapping.get(0)) != null) {
            moveDijkstra(
                    pointData.points.get(keyMapping.get(itemMapping.get(0)) - 1),
                    quaternions.points.get(keyMapping.get(itemMapping.get(0)) - 1)
            );
        }
        api.takeTargetItemSnapshot();
    }

    @Override
    protected void runPlan2() {
        // write your plan 2 here
        this.runPlan1();
    }

    @Override
    protected void runPlan3() {
        // write your plan 3 here
        this.runPlan1();
    }

    ArrayList<Point> concatPath(ArrayList<Point> move_oder) {
        ArrayList<Point> concatenated = new ArrayList<>();
        Area[] KOZs = new ConstAreas().KOZs;
        Point from = api.getRobotKinematics().getPosition();
        Log.i("StellarCoders", String.format("move_oder: %s", move_oder.toString()));
        for (int idx = 0; idx < move_oder.size(); idx++) {
            Point basePoint = new Point(from.getX(), from.getY(), from.getZ());
            int max_idx = idx;
            Point validDestination = new Point(basePoint.getX(), basePoint.getY(), basePoint.getZ());
            for (int next_idx = idx; next_idx < move_oder.size(); next_idx++) {
                boolean can = true;
                Point destination = new Point(move_oder.get(next_idx).getX(), move_oder.get(next_idx).getY(), move_oder.get(next_idx).getZ());
                for (Area koz : KOZs) {
                    for (Point[] ps : koz.getPolys()) {
                        boolean collisionCheckRet = Utils.isPolyLineCollision(basePoint, destination, ps);
                        if (collisionCheckRet) {
                            can = false;
                        }
                    }
                }
                if (can) {
                    if (max_idx < next_idx) {
                        max_idx = next_idx;
                        validDestination = new Point(destination.getX(), destination.getY(), destination.getZ());
                        Log.i("StellarCoders", String.format("Valid Route Updated! FROM: %s, TO: %s", basePoint, destination));

                    }
                }
            }
            idx = max_idx;
            Point to = move_oder.get(idx);
            concatenated.add(to);
            from = new Point(to.getX(), to.getY(), to.getZ());

        }

        return concatenated;
    }

    int moveDijkstra(Point goal, Quaternion q) {
        Kinematics kine = api.getRobotKinematics();
        Point currentPos = kine.getPosition();
        Log.i("StellarCoders", String.format("Current Pos %s", kine.getPosition().toString()));
        CheckPoints checkPoints = new CheckPoints();
        Dijkstra3D dijManager = new Dijkstra3D();
        ArrayList<Point> move_oder = dijManager.dijkstra(api.getRobotKinematics().getPosition(), goal);

        ArrayList<Point> concatenated = concatPath(move_oder);
        double totalDistance = 0.0;
        for (int i = 0; i < concatenated.size() - 1; i++) {
            totalDistance += Utils.distance3DSquare(concatenated.get(i), concatenated.get(i + 1));
        }
        Log.i("StellarCoders", String.format("Concatenated: %s", concatenated.toString()));
        double dist = 0.0;
        for (int i = 0; i < concatenated.size(); i++) {
            try {
                dist += Utils.distance3DSquare(currentPos, concatenated.get(i));
                Result result = api.moveTo(concatenated.get(i), Utils.QuatSlerp(kine.getOrientation(), q, Math.min(1, dist / totalDistance)), true);
                currentPos = concatenated.get(i);
            } catch (Error e) {
                Log.e("StellarCoders", e.getMessage());
                return -1;
            }
        }

        Log.i("StellarCoders", "Moved to Point");
        Log.i("StellarCoders", String.format("Current Pos %s", this.api.getRobotKinematics().getPosition().toString()));
        return 0;
    }
}

