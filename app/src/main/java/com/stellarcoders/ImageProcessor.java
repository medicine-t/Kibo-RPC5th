package com.stellarcoders;


import android.app.Application;
import android.graphics.Bitmap;
import android.content.Context;
import android.graphics.BitmapFactory;
import android.util.Log;

import com.stellarcoders.utils.Utils;

import org.tensorflow.lite.support.image.TensorImage;
import org.tensorflow.lite.task.core.BaseOptions;
import org.tensorflow.lite.task.vision.detector.Detection;
import org.tensorflow.lite.task.vision.detector.ObjectDetector;

import java.io.IOException;
import java.util.HashMap;
import java.util.List;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcApi;
import jp.jaxa.iss.kibo.rpc.defaultapk.GlobalContext;

import static org.tensorflow.lite.task.vision.detector.ObjectDetector.*;
import org.opencv.core.*;
import org.opencv.aruco.*;
import org.opencv.calib3d.Calib3d;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.HashMap;

public class ImageProcessor extends Application implements  IImageProcessor {
    /**
     * detectItems
     * ClassNum / Quantity
     * @return
     */

    ObjectDetector objectDetector = null;
    public ImageProcessor() {}

    private void init() {
        BaseOptions baseOptions = BaseOptions.builder()
                .setNumThreads(1)
                .build();
        ObjectDetectorOptions options = ObjectDetectorOptions.builder()
                .setScoreThreshold(0.3f)
                .setBaseOptions(baseOptions)
                .build();
        try {
            objectDetector = createFromFileAndOptions(GlobalContext.getInstance(),"model.tflite",options);
        } catch (IOException e) {
            throw new RuntimeException("Failed to make ObjectDetector",e);
        }
    }

    public HashMap<String, Integer> detectItems(KiboRpcApi api) {
        Log.i("StellarCoders[ImageProcessor::detectItems]","Object Detector Called");
        if (objectDetector == null) {
            this.init();
        }
        HashMap<String,Integer> result = new HashMap<>();
        Bitmap bitmapImage = Bitmap.createBitmap(1280,960,Bitmap.Config.ARGB_8888);
        org.opencv.android.Utils.matToBitmap(Utils.calibratedNavCam(api),bitmapImage);

        TensorImage image = TensorImage.fromBitmap(bitmapImage);
        List<Detection> detectResult = objectDetector.detect(image);
        Log.i("[StellarCoders]",detectResult.toString());

        for (Detection detect: detectResult) {
            String label = detect.getCategories().get(0).getLabel();
            Integer prevVal = result.get(label) != null ? result.get(label) : 0;
            result.put(label,prevVal + 1);
        }
        return result;
    }

    public List<Mat> extractTargetField(KiboRpcApi api) {
        Mat image = api.getMatNavCam();
        List<Mat> extractedImages = new ArrayList<>();

        // カメラのキャリブレーションパラメータ
        double[][] camStatistics = api.getNavCamIntrinsics();
        double[] camMtx = camStatistics[0];
        double[] dist = camStatistics[1];
        Mat navCam = api.getMatNavCam();
        Mat cameraMatrix = new Mat(3,3, CvType.CV_64FC1);
        cameraMatrix.put(0,0,camMtx);
        Mat distCoeffs = new Mat(1,5,CvType.CV_64FC1);
        distCoeffs.put(0,0,dist);

        // ARUCO辞書と検出器の設定
        Dictionary arucoDict = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);

        // ARUCOマーカーの検出
        MatOfInt ids = new MatOfInt();
        List<Mat> corners = new ArrayList<>();
        Aruco.detectMarkers(image, arucoDict, corners, ids);

        float markerLength = 0.05f;
        if (!ids.empty()) {
            // マーカーの位置と姿勢を推定
            Mat rvecs = new Mat();
            Mat tvecs = new Mat();
            Aruco.estimatePoseSingleMarkers(corners, markerLength, cameraMatrix, distCoeffs, rvecs, tvecs);

            Map<Integer, List<Point>> points = new HashMap<>();
            for (int i = 0; i < ids.rows(); i++) {
                int id = (int) ids.get(i, 0)[0];
                Mat localPoints = new Mat(4, 1, CvType.CV_32FC3);
                localPoints.put(0, 0, new float[]{(3.75f - 27) / 100, 3.75f / 100, 0});
                localPoints.put(1, 0, new float[]{(3.75f - 27) / 100, (3.75f - 15) / 100, 0});
                localPoints.put(2, 0, new float[]{3.75f / 100, 3.75f / 100, 0});
                localPoints.put(3, 0, new float[]{3.75f / 100, (3.75f - 15) / 100, 0});

                List<Point> pointList = new ArrayList<>();
                for (int j = 0; j < localPoints.rows(); j++) {
                    MatOfPoint2f imgPoints = new MatOfPoint2f();
                    Calib3d.projectPoints(new MatOfPoint3f(localPoints.row(j)), rvecs.row(i), tvecs.row(i), cameraMatrix, new MatOfDouble(distCoeffs), imgPoints);
                    pointList.add(new Point(imgPoints.get(0, 0)));
                }
                points.put(id, pointList);
            }

            // 歪み補正
            Mat undistortedImage = new Mat();
            Calib3d.undistort(image, undistortedImage, cameraMatrix, distCoeffs);

            int newWidth = 270;
            int newHeight = 150;
            for (Map.Entry<Integer, List<Point>> entry : points.entrySet()) {
                int id = entry.getKey();
                List<Point> pointList = entry.getValue();
                MatOfPoint2f srcPoints = new MatOfPoint2f(pointList.toArray(new Point[0]));

                // Perspective transformation
                MatOfPoint2f dstPoints = new MatOfPoint2f(
                        new Point(0, 0), new Point(0, newHeight), new Point(newWidth, 0), new Point(newWidth, newHeight)
                );
                Mat perspectiveTransform = Imgproc.getPerspectiveTransform(srcPoints, dstPoints);
                Mat plateImg = new Mat();
                Imgproc.warpPerspective(undistortedImage, plateImg, perspectiveTransform, new Size(newWidth, newHeight));

                // Save or return the extracted image
                extractedImages.add(plateImg);
            }
        } else {
            System.out.println("ARUCOマーカーが見つかりませんでした");
        }

        return extractedImages;
    }
}
