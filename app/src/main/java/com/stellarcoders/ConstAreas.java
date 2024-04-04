package com.stellarcoders;


public class ConstAreas {

    private static final float buffer = 0.10f; //KOZから離れるバッファ
    public static final Area[] KOZs = {
            new Area(10.87f - buffer, -9.5f - buffer, 4.27f - buffer, 11.6f + buffer, -9.45f + buffer, 4.97f + buffer),
            new Area(10.25f - buffer, -9.5f - buffer, -4.97f - buffer, -10.87f + buffer, -9.45f + buffer, -5.62f + buffer),
            new Area(10.87f - buffer,-8.5f - buffer,4.97f - buffer,11.6f + buffer,-8.45f + buffer,5.62f + buffer),
            new Area(10.25f - buffer,-8.5f - buffer,4.27f - buffer,10.7f + buffer,-8.45f + buffer,4.97f + buffer),
            new Area(10.87f - buffer,-7.40f - buffer,4.27f - buffer,11.6f + buffer,-7.35f + buffer,4.97f + buffer),
            new Area(10.25f - buffer,-7.40f - buffer,4.97f - buffer,10.87f + buffer,-7.35f + buffer,5.62f + buffer)
    };

    public final Area[] KIZs = {
            new Area(10.3f, -10.2f, 4.32f, 11.55f, -6.0f, 5.57f),
            new Area(9.5f, -10.5f, 4.02f, 10.5f, -9.6f, 4.8f),
    };
}
