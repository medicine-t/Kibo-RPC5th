package com.stellarcoders;


import android.app.Application;
import android.graphics.Bitmap;
import android.content.Context;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.RectF;
import android.util.Log;

import com.stellarcoders.utils.Utils;

import org.tensorflow.lite.support.image.TensorImage;
import org.tensorflow.lite.task.core.BaseOptions;
import org.tensorflow.lite.task.vision.classifier.Classifications;
import org.tensorflow.lite.task.vision.classifier.ImageClassifier;
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

//    ObjectDetector objectDetector = null;
    ImageClassifier imageClassifier = null;
    public ImageProcessor() {}

    private void init() {
        BaseOptions baseOptions = BaseOptions.builder()
                .setNumThreads(1)
                .build();
//        ObjectDetectorOptions options = ObjectDetectorOptions.builder()
//                .setScoreThreshold(0.3f)
//                .setBaseOptions(baseOptions)
//                .build();
        ImageClassifier.ImageClassifierOptions classifierOptions = ImageClassifier.ImageClassifierOptions.builder()
                .setBaseOptions(baseOptions)
                .setMaxResults(1)
                .build();
        try {
//            objectDetector = createFromFileAndOptions(GlobalContext.getInstance(),"model_detector.tflite",options);
            imageClassifier = ImageClassifier.createFromFileAndOptions(GlobalContext.getInstance(),"model_quantized.classifier.tflite",classifierOptions);
        } catch (IOException e) {
            throw new RuntimeException("Failed to make ObjectDetector",e);
        }
    }

    private final int extractedWidth = 270;
    private final int extractedHeight = 150;

    public static Bitmap cropAndResizeBitmap(Bitmap bitmap, RectF rectF, int targetWidth, int targetHeight) {
        // RectFを元にビットマップをクロップ
        Bitmap croppedBitmap = Bitmap.createBitmap(
                bitmap,
                (int) Math.max(0,rectF.left),
                (int) Math.max(0,rectF.top),
                (int) rectF.width(),
                (int) rectF.height()
        );

        // アスペクト比を保ったまま指定サイズにリサイズ
        float aspectRatio = (float) croppedBitmap.getWidth() / (float) croppedBitmap.getHeight();
        int width, height;
        if (croppedBitmap.getWidth() > croppedBitmap.getHeight()) {
            width = targetWidth;
            height = Math.round(targetWidth / aspectRatio);
        } else {
            height = targetHeight;
            width = Math.round(targetHeight * aspectRatio);
        }

        Bitmap scaledBitmap = Bitmap.createScaledBitmap(croppedBitmap, width, height, true);

        // 余った部分を白で埋める
        Bitmap finalBitmap = Bitmap.createBitmap(targetWidth, targetHeight, Bitmap.Config.ARGB_8888);
        Canvas canvas = new Canvas(finalBitmap);
        canvas.drawColor(Color.GRAY);

        // 中央に配置
        float left = (targetWidth - width) / 2f;
        float top = (targetHeight - height) / 2f;
        canvas.drawBitmap(scaledBitmap, left, top, new Paint());

        return finalBitmap;
    }

    @Override
    public HashMap<String, Integer> detectItems(Mat matImage) {
        Log.i("StellarCoders[ImageProcessor::detectItems]","Object Detector Called");
        if (imageClassifier == null) {
            this.init();
        }
        // crop [0,0,200,170]
        Rect cropRegion = new Rect(
                0,0,200,150
        );
        matImage = matImage.submat(cropRegion);

        HashMap<String,Integer> result = new HashMap<>();
        Bitmap bitmapImage = Bitmap.createBitmap(200,150,Bitmap.Config.ARGB_8888);
        org.opencv.android.Utils.matToBitmap(matImage,bitmapImage);

        TensorImage image = TensorImage.fromBitmap(bitmapImage);
        List<Classifications> classificationResult = imageClassifier.classify(image);
        String label = classificationResult.get(0).getCategories().get(0).getLabel();
        Log.i("[StellarCoders]::ClassificationResult",String.format("label: %s, prob: %f",label,classificationResult.get(0).getCategories().get(0).getScore()));
        int numObj = ImageSegmentation.segmentImage(matImage);
        result.put(label,numObj);
        Log.i("[StellarCoders]::ClassificationResult",String.format("ImageSegmentation.segmentImage: %d",numObj));
        Log.i("[StellarCoders]",result.toString());
        return result;
    }

//    public HashMap<String, Integer> _detectItems(Mat matImage) {
//        Log.i("StellarCoders[ImageProcessor::detectItems]","Object Detector Called");
//        if (objectDetector == null) {
//            this.init();
//        }
//        HashMap<String,Integer> result = new HashMap<>();
//        Bitmap bitmapImage = Bitmap.createBitmap(extractedWidth,extractedHeight,Bitmap.Config.ARGB_8888);
//        org.opencv.android.Utils.matToBitmap(matImage,bitmapImage);
//
//        TensorImage image = TensorImage.fromBitmap(bitmapImage);
//        List<Detection> detectResult = objectDetector.detect(image);
//
//
//        for (Detection detect: detectResult) {
//            if (detect.getBoundingBox().left > 200) {
//                continue;
//            }
//
//            Bitmap croppedImage = cropAndResizeBitmap(bitmapImage,detect.getBoundingBox(),224,224);
//            TensorImage croppedTensorImage = TensorImage.fromBitmap(croppedImage);
//            List<Classifications> classificateResult = imageClassifier.classify(croppedTensorImage);
//            for(Classifications classifications: classificateResult) {
//                String label = classifications.getCategories().get(0).getLabel();
//                int prevVal = result.get(label) != null ? result.get(label) : 0;
//                result.put(label,prevVal + 1);
//            }
//        }
//        Log.i("[StellarCoders]",result.toString());
//        return result;
//    }

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
                localPoints.put(3, 0, new float[]{3.75f  / 100, (3.75f - 15) / 100, 0});

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

            int newWidth = extractedWidth;
            int newHeight = extractedHeight;
            for (Map.Entry<Integer, List<Point>> entry : points.entrySet()) {
                int id = entry.getKey();
                List<Point> pointList = entry.getValue();
                MatOfPoint2f srcPoints = new MatOfPoint2f(pointList.toArray(new Point[0]));

                // Check if point in image area
                int width = 1280;
                int height = 960;
                boolean usable = true;
                for(Point point : entry.getValue()){
                    if (point.x < 0 || point.x >= width) {
                        usable = false;
                    }else if (point.y < 0 || point.y >= height) {
                        usable = false;
                    }
                }

                if (!usable) {
                    continue;
                }

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
