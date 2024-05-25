package com.stellarcoders;

import org.opencv.android.Utils;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Mat;

import android.graphics.Bitmap;
import android.util.Log;

import java.util.Random;

public class ImageSegmentation {

    public static int segmentImage(Mat image) {
        // グレースケールに変換
        Mat gray = new Mat();
        // 入力画像がカラーかグレースケールかをチェック
        if (image.channels() == 3) {
            // カラー画像の場合、グレースケールに変換
            Imgproc.cvtColor(image, gray, Imgproc.COLOR_BGR2GRAY);
        } else {
            // グレースケール画像の場合、そのまま使用
            gray = image;
        }

        // 二値化
        Mat binary = new Mat();
        Imgproc.threshold(gray, binary, 100, 255, Imgproc.THRESH_BINARY_INV);

        // 輪郭を検出
        Mat hierarchy = new Mat();
        java.util.List<MatOfPoint> contours = new java.util.ArrayList<>();
        Imgproc.findContours(binary, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // オブジェクト内部を塗りつぶし
        for (MatOfPoint contour : contours) {
            Imgproc.drawContours(binary, java.util.Collections.singletonList(contour), -1, new Scalar(255), Core.FILLED);
        }

        // ノイズを除去するためにガウシアンブラーを適用
        Mat blurred = new Mat();
        Imgproc.GaussianBlur(binary, blurred, new Size(5, 5), 0);

        // 確実なバックグラウンド領域を見つける
        Mat sureBg = new Mat();
        Imgproc.dilate(blurred, sureBg, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3)), new Point(-1, -1), 3);

        // 確実な前景領域を見つける
        Mat distTransform = new Mat();
        Imgproc.distanceTransform(blurred, distTransform, Imgproc.DIST_L2, 5);
        Mat sureFg = new Mat();
        Imgproc.threshold(distTransform, sureFg, 0.5 * Core.minMaxLoc(distTransform).maxVal, 255, 0);

        // 不確実な領域を見つける
        sureFg.convertTo(sureFg, CvType.CV_8U);
        Mat unknown = new Mat();
        Core.subtract(sureBg, sureFg, unknown);

        // ラベリング
        Mat markers = new Mat();
        Imgproc.connectedComponents(sureFg, markers);

        // ラベルに1を加える（背景を0から1に、他を1以上にするため）
        for (int i = 0; i < markers.rows(); i++) {
            for (int j = 0; j < markers.cols(); j++) {
                markers.put(i, j, markers.get(i, j)[0] + 1);
            }
        }

        // 未知の領域を0にマーク
        for (int i = 0; i < unknown.rows(); i++) {
            for (int j = 0; j < unknown.cols(); j++) {
                if (unknown.get(i, j)[0] == 255) {
                    markers.put(i, j, 0);
                }
            }
        }

        // マーカーを32ビット整数型に変換
        markers.convertTo(markers, CvType.CV_32SC1);
        Imgproc.cvtColor(image,image, Imgproc.COLOR_GRAY2BGR);

        // Watershedアルゴリズムを適用
        Imgproc.watershed(image, markers);

        // オブジェクトの数を計算

        return (int) (Core.minMaxLoc(markers).maxVal - 1);
    }
}
