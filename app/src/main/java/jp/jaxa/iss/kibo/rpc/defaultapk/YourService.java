package jp.jaxa.iss.kibo.rpc.defaultapk;

import com.stellarcoders.Area;
import com.stellarcoders.CheckPoints;
import com.stellarcoders.ConstAreas;
import com.stellarcoders.ConstPoints;
import com.stellarcoders.ConstQuaternions;
import com.stellarcoders.ImageProcessor;
import com.stellarcoders.utils.Dijkstra3D;
import com.stellarcoders.utils.Utils;
import com.stellarcoders.utils.Vector3;

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

        // ゴールまで移動してそれぞれのターゲットを認識する
        ConstPoints pointData = new ConstPoints();
        ConstQuaternions quaternions = new ConstQuaternions();
        Thread thread = new Thread(() -> {
            //TODO: 移動順の最適化
            for (int i = 0; i < 4; i++) {
                int index = i;
                moveDijkstra(pointData.points.get(index), quaternions.points.get(index));
            }
        });
        thread.start();

        // Get a camera image.
        int cnt = 0;
        ImageProcessor imageProcessor = new ImageProcessor();
        HashMap<Integer,String> itemMapping = new HashMap<>();
        HashMap<String,Integer> keyMapping = new HashMap<>();

        HashMap<String,ArrayList<Integer>> detectionResult = new HashMap<>();

        while (thread.isAlive()) {
            List<Mat> fields = imageProcessor.extractTargetField(api);
            for (Mat field : fields){
                if(cnt % 10 == 0)api.saveMatImage(field, String.format("ExtractImage_%d.png",cnt));
                HashMap<String, Integer> result = imageProcessor.detectItems(field);
                List<Integer> ids = Utils.searchMarker(field);
                int labelCount = result.keySet().size();
                if (labelCount == 1 && ids.size() == 1){
                    for(Map.Entry<String,Integer> entry : result.entrySet()) {
                        itemMapping.put(ids.get(0) - 100, entry.getKey());
                        if(ids.get(0) - 100 != 0) {
                            keyMapping.put(entry.getKey(),ids.get(0) - 100);
                            ArrayList<Integer> array = detectionResult.getOrDefault(entry.getKey(),new ArrayList<>());
                            array.add(entry.getValue());
                            detectionResult.put(entry.getKey(),array);
                        }
                    }
                }
            }
            cnt++;
        }
        // 各キーについて最頻値を出す
        // 報告
        for(Map.Entry<String,ArrayList<Integer>> entry: detectionResult.entrySet()) {
            int id = keyMapping.getOrDefault(entry.getKey(),1);
            // 最頻値
            int mode = Utils.getMode(entry.getValue());
            api.setAreaInfo(id,entry.getKey(), mode);
        }
        if(thread.isAlive()) ;thread.interrupt();
        // 報告
        Thread moveToAstronaut = new Thread(() -> {
            moveDijkstra(pointData.goal, quaternions.goal);
            api.reportRoundingCompletion();
            try {
                Thread.sleep(1000 * 60 * 2);
            }catch (InterruptedException e) {
                Log.i("StellarCoders","interrupted (to astronaut)");
            }
        });
        moveToAstronaut.start();
        boolean alive = true;
        while (moveToAstronaut.isAlive()){
            List<Mat> fields = imageProcessor.extractTargetField(api);
            for (Mat field : fields){
                api.saveMatImage(field, String.format("ExtractImage_%d.png",cnt));
                HashMap<String, Integer> result = imageProcessor.detectItems(field);
                List<Integer> ids = Utils.searchMarker(field);
                int labelCount = result.keySet().size();
                if (labelCount == 1 && ids.size() == 1){
                    for(Map.Entry<String,Integer> entry : result.entrySet()) {
                        itemMapping.put(ids.get(0) - 100, entry.getKey());
                        if(ids.get(0) - 100 == 0) {
                            api.notifyRecognitionItem();
                            moveToAstronaut.interrupt();
                            alive = false;
                            break;
                        }
                    }
                }
            }
            if(!alive)break;
        }

        // ゴールの目的地へ移動

        Log.i("StellarCoders","move to goal");
        api.saveMatImage(Utils.calibratedNavCam(api), String.format("Image%d.png",cnt));
        Log.i("StellarCoders",String.format("%s,%s",keyMapping,itemMapping));
        if(itemMapping.get(0) != null && keyMapping.get(itemMapping.get(0)) != null) {
            int targetIndex = keyMapping.get(itemMapping.get(0)) - 1;
            // 大雑把な移動
            moveDijkstra(
                    pointData.points.get(targetIndex),
                    quaternions.points.get(targetIndex)
            );

            Vector3 rel = Utils.getDiffFromCam(api,targetIndex);
            //Point currentPosition = api.getRobotKinematics().getPosition();
            Log.i("StellarCoders",String.format("relative %.3f, %.3f, %.3f",rel.getX(),rel.getY(),rel.getZ()));
            if (rel.getX() * rel.getX() + rel.getY() * rel.getY() + rel.getZ() * rel.getZ() > 0.01){
                api.relativeMoveTo(new Point(rel.getX(),rel.getY(),rel.getZ()),quaternions.points.get(targetIndex),true);
            }

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

    void moveDijkstra(Point goal, Quaternion q) {
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
                return;
            }
        }

        Log.i("StellarCoders", "Moved to Point");
        Log.i("StellarCoders", String.format("Current Pos %s", this.api.getRobotKinematics().getPosition().toString()));
    }
}

