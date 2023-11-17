package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Typeface;
import android.text.TextPaint;
import android.util.Pair;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Locale;
import java.util.Objects;
import java.util.concurrent.atomic.AtomicReference;

public class TeamPropProcessor implements VisionProcessor, CameraStreamSource {

    private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));
    private final HashMap<PropColor, Pair<Scalar, Scalar>> thresholds;
    private final ArrayList<MatOfPoint> contours;
    private final Mat hierarchy = new Mat();
    private final TextPaint textPaint;
    private final Paint linePaint;
    private final boolean showMask;
    private final PropColor propColor;
    private Double left = 200.;
    private Double right = 400.;
    private Double minArea = 4000.;
    private Double largestContourX, largestContourY, largestContourArea;
    private MatOfPoint largestContour;
    private Location currentLocation = Location.NOT_FOUND;

    public TeamPropProcessor(@NonNull PropColor propColor, Double minArea, Double left, Double right) {
        this(propColor, minArea, left, right, false);
    }
    public TeamPropProcessor(@NonNull PropColor propColor, Double minArea, Double left, Double right, boolean showMask) {
        this.propColor = propColor;
        this.left = left;
        this.right = right;
        this.minArea = minArea;
        this.showMask = showMask;
        this.contours = new ArrayList<>();

        thresholds = new HashMap<PropColor, Pair<Scalar, Scalar>>() {{
            put(PropColor.RED, new Pair<>(new Scalar(110, 70, 50), new Scalar(150, 255, 255)));
            put(PropColor.BLUE, new Pair<>(new Scalar(0, 50, 30), new Scalar(30, 255, 150)));
        }};

        textPaint = new TextPaint();
        textPaint.setColor(Color.GREEN);
        textPaint.setTextAlign(Paint.Align.CENTER);
        textPaint.setAntiAlias(true);
        textPaint.setTextSize(40);
        textPaint.setTypeface(Typeface.DEFAULT_BOLD);

        linePaint = new Paint();
        linePaint.setColor(Color.RED);
        linePaint.setAntiAlias(true);
        linePaint.setStrokeWidth(10);
        linePaint.setStrokeCap(Paint.Cap.ROUND);
        linePaint.setStrokeJoin(Paint.Join.ROUND);
    }

    public void close() {
        hierarchy.release();
    }

    public Pair<Scalar, Scalar> getThresholds(PropColor color) {
        return thresholds.get(color);
    }

    public void setThresholds(PropColor color, Scalar min_values, Scalar max_values) {
        thresholds.replace(color, new Pair<>(min_values, max_values));
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Mat hsv = new Mat();
        frame.copyTo(hsv);

        Imgproc.cvtColor(hsv, hsv, Imgproc.COLOR_BGR2HSV);
        Core.inRange(hsv, Objects.requireNonNull(thresholds.get(propColor)).first, Objects.requireNonNull(thresholds.get(propColor)).second, hsv);
        contours.clear();

        Imgproc.findContours(hsv, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        largestContourArea = -1.0;
        largestContour = null;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > largestContourArea && area > minArea) {
                largestContourArea = area;
                largestContour = contour;
            }
        }

        largestContourX = largestContourY = -1.0;

        if (largestContour != null) {
            Moments moment = Imgproc.moments(largestContour);
            largestContourX = (moment.m10 / moment.m00);
            largestContourY = (moment.m01 / moment.m00);

            if (largestContourX < left)
                currentLocation = Location.LEFT;
            else if (largestContourX > right)
                currentLocation = Location.RIGHT;
            else currentLocation = Location.MIDDLE;
        }

        Bitmap b = (!showMask ? Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565) : Bitmap.createBitmap(hsv.width(), hsv.height(), Bitmap.Config.RGB_565));
        Utils.matToBitmap((showMask ? hsv : frame), b);
        lastFrame.set(b);
        return showMask ? hsv : frame;
    }

    public Double getLargestContourArea() {
        return largestContourArea;
    }

    public Location getCurrentLocation() {
        return currentLocation;
    }

    public Pair<Double, Double> getLargestContourCoords() {
        return new Pair<>(largestContourX, largestContourY);
    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        if (largestContour != null) {
            Rect rect = Imgproc.boundingRect(largestContour);

            float[] points = {rect.x * scaleBmpPxToCanvasPx, rect.y * scaleBmpPxToCanvasPx, (rect.x + rect.width) * scaleBmpPxToCanvasPx, (rect.y + rect.height) * scaleBmpPxToCanvasPx};

            canvas.drawLine(points[0], points[1], points[0], points[3], linePaint);
            canvas.drawLine(points[0], points[1], points[2], points[1], linePaint);

            canvas.drawLine(points[0], points[3], points[2], points[3], linePaint);
            canvas.drawLine(points[2], points[1], points[2], points[3], linePaint);

            String text = String.format(Locale.ENGLISH, "%s", currentLocation.toString());

            canvas.drawText(text, largestContourX.floatValue() * scaleBmpPxToCanvasPx, largestContourY.floatValue() * scaleBmpPxToCanvasPx, textPaint);
        }
    }

    public enum PropColor {
        BLUE,
        RED
    }

    public enum Location {
        NOT_FOUND,
        LEFT,
        MIDDLE,
        RIGHT
    }
}
