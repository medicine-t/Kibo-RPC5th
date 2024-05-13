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
            String label = detect.getCategories().get(0).getDisplayName();
            Integer prevVal = result.get(label) != null ? result.get(label) : 0;
            result.put(label,prevVal + 1);
        }
        return result;
    }
}
