package com.stellarcoders;

import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.HashMap;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcApi;

public interface IImageProcessor {

    /**
     * detectItems
     * ClassNum / Quantity
     * @return
     */
    HashMap<String, Integer> detectItems(Mat matImage) ;
}
