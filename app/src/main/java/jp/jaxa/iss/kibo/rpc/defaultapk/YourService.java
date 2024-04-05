package jp.jaxa.iss.kibo.rpc.defaultapk;

import com.stellarcoders.Area;
import com.stellarcoders.CheckPoints;
import com.stellarcoders.ConstAreas;
import com.stellarcoders.ConstPoints;
import com.stellarcoders.ConstQuaternions;
import com.stellarcoders.utils.Dijkstra3D;
import com.stellarcoders.utils.Utils;

import android.annotation.SuppressLint;
import android.util.Log;

import org.opencv.core.Mat;

import java.util.ArrayList;

import gov.nasa.arc.astrobee.Kinematics;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {
    @SuppressLint("DefaultLocale")
    @Override
    protected void runPlan1(){
        // write your plan 1 here
        api.startMission();

        ConstPoints pointData = new ConstPoints();
        ConstQuaternions quaternions = new ConstQuaternions();

        //TODO: 移動順の最適化
        for(int i = 0;i < 4;i++){
            int index = i;
            Thread thread = new Thread(() -> { moveDijkstra(pointData.points.get(index),quaternions.points.get(index));});
            thread.start();
            // Get a camera image.
            int cnt = 0;
            while(thread.isAlive()) {
                Mat image = api.getMatNavCam();
                api.saveMatImage(image, String.format("Image%d-%d.png", i,cnt));
                cnt++;
            }
            /* *********************************************************************** */
            /* Write your code to recognize type and number of items in the each area! */
            /* *********************************************************************** */

            // When you recognize items, let’s set the type and number.
            api.setAreaInfo(1, "item_name", 1);
        }

        // When you move to the front of the astronaut, report the rounding completion.
        api.reportRoundingCompletion();

        moveDijkstra(pointData.goal,quaternions.goal);
        api.notifyRecognitionItem();

        api.takeTargetItemSnapshot();
    }

    @Override
    protected void runPlan2(){
        // write your plan 2 here
        this.runPlan1();
    }

    @Override
    protected void runPlan3(){
        // write your plan 3 here
        this.runPlan1();
    }

    ArrayList<Point> concatPath(ArrayList<Point> move_oder){
        ArrayList<Point> concatenated = new ArrayList<>();
        Area[] KOZs = new ConstAreas().KOZs;
        Point from = api.getRobotKinematics().getPosition();
        Log.i("StellarCoders",String.format("move_oder: %s",move_oder.toString()));
        for(int idx = 0;idx < move_oder.size();idx++){
            Point basePoint = new Point(from.getX(), from.getY(),from.getZ());
            int max_idx = idx;
            Point validDestination = new Point(basePoint.getX(),basePoint.getY(),basePoint.getZ());
            for(int next_idx = idx;next_idx < move_oder.size();next_idx++){
                boolean can = true;
                Point destination = new Point(move_oder.get(next_idx).getX(),move_oder.get(next_idx).getY(),move_oder.get(next_idx).getZ());
                for(Area koz: KOZs){
                    for (Point[] ps: koz.getPolys()){
                        boolean collisionCheckRet = Utils.isPolyLineCollision(basePoint,destination,ps);
                        if(collisionCheckRet) {
                            can = false;
                        }
                    }
                }
                if(can){
                    if(max_idx < next_idx) {
                        max_idx = next_idx;
                        validDestination = new Point(destination.getX(),destination.getY(),destination.getZ());
                        Log.i("StellarCoders",String.format("Valid Route Updated! FROM: %s, TO: %s",basePoint,destination));

                    }
                }
            }
            idx = max_idx;
            Point to = move_oder.get(idx);
            concatenated.add(to);
            from = new Point(to.getX(),to.getY(),to.getZ());

        }

        return  concatenated;
    }

    int moveDijkstra(Point goal, Quaternion q) {
        Kinematics kine = api.getRobotKinematics();
        Point currentPos = kine.getPosition();
        Log.i("StellarCoders",String.format("Current Pos %s",kine.getPosition().toString()));
        CheckPoints checkPoints = new CheckPoints();
        Dijkstra3D dijManager = new Dijkstra3D();
        ArrayList<Point> move_oder = dijManager.dijkstra(api.getRobotKinematics().getPosition(),goal);

        ArrayList<Point> concatenated = concatPath(move_oder);
        double totalDistance = 0.0;
        for (int i = 0; i < concatenated.size() - 1; i++) {
            totalDistance += Utils.distance3DSquare(concatenated.get(i),concatenated.get(i + 1));
        }
        Log.i("StellarCoders",String.format("Concatenated: %s",concatenated.toString()));
        double dist = 0.0;
        for (int i = 0; i < concatenated.size(); i++) {
            try{
                dist += Utils.distance3DSquare(currentPos,concatenated.get(i));
                api.moveTo(concatenated.get(i),Utils.QuatSlerp(kine.getOrientation(),q,Math.min(1,dist / totalDistance)), true);
                currentPos = concatenated.get(i);
            }catch (Error e){
                Log.e("StellarCoders",e.getMessage());
                return -1;
            }
        }

        Log.i("StellarCoders","Moved to Point");
        Log.i("StellarCoders",String.format("Current Pos %s",this.api.getRobotKinematics().getPosition().toString()));
        return 0;
    }
}

