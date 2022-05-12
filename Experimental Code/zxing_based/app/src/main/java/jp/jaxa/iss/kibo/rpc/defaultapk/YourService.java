package jp.jaxa.iss.kibo.rpc.defaultapk;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import android.graphics.Bitmap;
import android.os.SystemClock;
import android.util.Log;
// android library
import com.google.zxing.BinaryBitmap;
import com.google.zxing.DecodeHintType;
import com.google.zxing.LuminanceSource;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.qrcode.QRCodeReader;
// zxing library
import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {

    @Override
    protected void runPlan1() {
        // astrobee is undocked and the mission starts
        api.startMission();

        //Point A
        moveToWrapper(11.21, -9.8, 4.79, 0, 0, -0.707, 0.707);
        Log.d("DevLog_status", "moved to point a");

        double[] QRResult = readQR(3);
        Log.d("DevLog_status_QRvalue", QRResult[0] + ", " + QRResult[1] + ", " + QRResult[2] + ", " + QRResult[3]);
        if (QRResult[0] == 5.0) {
            moveToWrapper(QRResult[1] - 0.3, QRResult[2], 4.79, 0, 0, -0.707, 0.707);
            moveToWrapper(QRResult[1] - 0.3, QRResult[2], QRResult[3] - 0.075, 0, 0, -0.707, 0.707);
            moveToWrapper(QRResult[1], QRResult[2], QRResult[3], 0, 0, -0.707, 0.707);
        } else if (QRResult[0] == 6.0) {
            moveToWrapper(QRResult[1] - 0.3, QRResult[2], 4.79, 0, 0, -0.707, 0.707);
            moveToWrapper(QRResult[1] - 0.3, QRResult[2], QRResult[3], 0, 0, -0.707, 0.707);
            moveToWrapper(QRResult[1], QRResult[2], QRResult[3], 0, 0, -0.707, 0.707);
        } else if (QRResult[0] == 7.0) {
            double initialX = QRResult[1] + 0.3;
            // if place to go is outside keep in zone
            if (initialX > 11.54) {
                moveToWrapper(11.54, QRResult[2], 4.79, 0, 0, -0.707, 0.707);
                moveToWrapper(11.54, QRResult[2], QRResult[3]-0.075, 0, 0, -0.707, 0.707);
            } else {
                moveToWrapper(QRResult[1] + 0.3, QRResult[2], 4.79, 0, 0, -0.707, 0.707);
                moveToWrapper(QRResult[1] + 0.3, QRResult[2], QRResult[3]-0.075, 0, 0, -0.707, 0.707);
            }
            moveToWrapper(QRResult[1], QRResult[2], QRResult[3], 0, 0, -0.707, 0.707);
        } else {
            moveToWrapper(QRResult[1], QRResult[2], QRResult[3], 0, 0, -0.707, 0.707);
        }

        double[] ar = calTargetPos(3);
        // irradiate the laser and take snapshots
        api.laserControl(true);
        api.takeSnapshot();

        ///////// move to point B //////////
        if (QRResult[0] == 1){
            moveToWrapper(QRResult[1] - 0.075, QRResult[2], QRResult[3] - 0.3, 0, 0, -0.707, 0.707);
            QRResult[1] = QRResult[1] - 0.075;
            QRResult[3] = QRResult[3] - 0.3;
            double IdealXYSlope = (QRResult[1] - 10.6)/(QRResult[2]+8.0);
            double YAtKOZ = (IdealXYSlope * (-8.8-QRResult[2]))+QRResult[1];
            //if can go directly without turning
            if (YAtKOZ < 10.7){
                moveToWrapper(10.6, -8.0, 4.5, 0, 0, -0.707, 0.707);
                Log.d("PATH_took", "can go directly");
            }
            //else hit KOZ
            else{
                //slope from target point to edge of keep out zone.
                double secondPathSlope = (QRResult[1] - 10.7)/(QRResult[2] + 8.8);
                double secondY = ((10.6 - QRResult[1])/secondPathSlope) + QRResult[2];

                //Calculate second Z using similar triangle
                double secondZ = QRResult[3] - (((QRResult[3] - 4.5) * (QRResult[2] - secondY))/(QRResult[2] + 8.0));
                moveToWrapper(10.6, secondY, secondZ, 0, 0, -0.707, 0.707);
                moveToWrapper(10.6, -8.0, 4.5, 0, 0, -0.707, 0.707);
                Log.d("PATH_took", "take 1 turn");
            }

        }

        else if (QRResult[0] == 2 || QRResult[0] == 3){
            double IdealXYSlope = (QRResult[1] - 10.6)/(QRResult[2]+8.0);
            double YAtKOZ = (IdealXYSlope * (-8.8-QRResult[2]))+QRResult[1];
            //if can go directly without turning
            if (YAtKOZ < 10.7){
                moveToWrapper(10.6, -8.0, 4.5, 0, 0, -0.707, 0.707);
                Log.d("PATH_took", "can go directly");
            }
            //else hit KOZ
            else{
                //slope from target point to edge of keep out zone.
                double secondPathSlope = (QRResult[1] - 10.7)/(QRResult[2] + 8.8);
                double secondY = ((10.6 - QRResult[1])/secondPathSlope) + QRResult[2];

                //Calculate second Z using similar triangle
                double secondZ = QRResult[3] - (((QRResult[3] - 4.5) * (QRResult[2] - secondY))/(QRResult[2] + 8.0));
                moveToWrapper(10.6, secondY, secondZ, 0, 0, -0.707, 0.707);
                moveToWrapper(10.6, -8.0, 4.5, 0, 0, -0.707, 0.707);
                Log.d("PATH_took", "take 1 turn");
            }
        }

        // Send mission completion
        api.reportMissionCompletion();
    }

    @Override
    protected void runPlan2() {
        // write here your plan 2
    }

    @Override
    protected void runPlan3() {
        // write here your plan 3
    }

    // You can add your method
    private boolean moveToWrapper(double pos_x, double pos_y, double pos_z,
                                  double qua_x, double qua_y, double qua_z,
                                  double qua_w) {

        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float) qua_x, (float) qua_y,
                (float) qua_z, (float) qua_w);

        Result result = api.moveTo(point, quaternion, true);

        int loopCounter = 0;
        while (!result.hasSucceeded() && loopCounter < 3) {
            result = api.moveTo(point, quaternion, true);
            loopCounter++;
        }

        Log.d("move[count]", ""+loopCounter);
        return true;
    }

    public double[] readQR(int count_max) {
        Log.d("lib_used", "zxing qr only");
        String contents = null;
        int count = 0;
        double pattern = 0, final_x = 0, final_y = 0, final_z = 0;

        long start_time = SystemClock.elapsedRealtime();
        while (contents == null && count < count_max) {
            Log.d("DevLog_status_QR[status]:", " start");

            Bitmap bMap = Bitmap.createBitmap(api.getBitmapNavCam(), 600, 600, 200, 200);
            int[] intArray = new int[bMap.getWidth() * bMap.getHeight()];
            bMap.getPixels(intArray, 0, bMap.getWidth(), 0, 0, bMap.getWidth(), bMap.getHeight());
            LuminanceSource source = new RGBLuminanceSource(bMap.getWidth(), bMap.getHeight(), intArray);
            BinaryBitmap bitmap = new BinaryBitmap(new HybridBinarizer(source));
            Log.d("DevLog_status_QR[status]:", "done preprocess");

            Map<DecodeHintType, String> hints = new HashMap<>();
            hints.put(DecodeHintType.CHARACTER_SET, "utf-8");
            hints.put(DecodeHintType.POSSIBLE_FORMATS, "QR_CODE");

            try {
                com.google.zxing.Result result = new QRCodeReader().decode(bitmap, hints);
                contents = result.getText();
                Log.d("DevLog_status_QR[status]:", " Detected");
                api.sendDiscoveredQR(contents);

                String[] multi_contents = contents.split(",");
                pattern = Double.parseDouble(multi_contents[0].split(":")[1]);
                final_x = Double.parseDouble(multi_contents[1].split(":")[1]);
                final_y = Double.parseDouble(multi_contents[2].split(":")[1]);
                String tmp = multi_contents[3].split(":")[1];
                final_z = Double.parseDouble(tmp.substring(0, tmp.length() - 1));
            } catch (Exception e) {
                Log.e("DevLog_status_QR[status]:", " Not detected", e);
            }
            //////////////////////////////////////////////////////////////////////////////////////////////////////
            count++;
        }
        Log.d("DevLog_status_QR[status]:", " stop");
        long stop_time = SystemClock.elapsedRealtime();

        Log.d("DevLog_status_QR[count]:", " " + count);
        Log.d("DevLog_status_QR[total_time]:", " " + (stop_time - start_time) / 1000);

        return new double[]{pattern, final_x, final_y, final_z};
    }

    public double[] calTargetPos(int count_max) {
        int count = 0;
        boolean detected = false;
        double result[] = new double[2];
        long start_time = SystemClock.elapsedRealtime();
        while (!detected && count < count_max) {
            Log.d("AR[status]:", "start");
//            Mat source = api.getMatNavCam();
            Mat source = new Mat(api.getMatNavCam(), new Rect(200,200,960,660));
            Mat ids = new Mat();
            Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
            List<Mat> corners = new ArrayList<>();

            int topLeft = 0, bottomRight = 0;
            try {
                Log.d("AR[status]:", "start finding AR");
                Aruco.detectMarkers(source, dictionary, corners, ids);
                Log.d("AR[status]:", "finish finding AR");
                Log.d("AR[content]", ids.get(0, 0)[0] + ", " + ids.get(1, 0)[0] + ", " + ids.get(2, 0)[0] + ", " + ids.get(3, 0)[0]);

                for (int i = 0; i < 4; i++) {
                    double id = ids.get(i, 0)[0];
                    if(id == 2.0){
                        topLeft = i;
                    }
                    else if(id == 4.0){
                        bottomRight = i;
                    }
                }

                for (int i = 0; i < 4; i++) {
                    Log.d("topleft_before", corners.get(topLeft).get(0,i)[0] + ", " + corners.get(topLeft).get(0,i)[1]);
                }
                Mat corrected_topleft = new Mat(1,4,CvType.CV_32FC2);
                for (int i = 0; i < 4; i++) {
                    corrected_topleft.put(0,i,new double[]{corners.get(topLeft).get(0,i)[0] + 200.0, corners.get(topLeft).get(0,i)[1] + 200.0});
                }

                for (int i = 0; i < 4; i++) {
                    Log.d("topleft_after", corrected_topleft.get(0,i)[0] + ", " + corners.get(topLeft).get(0,i)[1]);
                }

                for (int i = 0; i < 4; i++) {
                    Log.d("bottomRight_before", corners.get(bottomRight).get(0,i)[0] + ", " + corners.get(bottomRight).get(0,i)[1]);
                }

                Mat corrected_bottomright = new Mat(1,4,CvType.CV_32FC2);
                for (int i = 0; i < 4; i++) {
                    corrected_bottomright.put(0,i,new double[]{corners.get(bottomRight).get(0,i)[0] + 200.0, corners.get(bottomRight).get(0,i)[1] +200.0});
                }

                for (int i = 0; i < 4; i++) {
                    Log.d("bottomRight_after", corrected_bottomright.get(0,i)[0] + ", " + corners.get(bottomRight).get(0,i)[1]);
                }


                double[][] cameraParam = api.getNavCamIntrinsics();
                Mat topleft_undistort = new Mat(1, 4, CvType.CV_32FC2);
                Mat bottomright_undistort = new Mat(1, 4, CvType.CV_32FC2);
                Mat cameraMatrix = new Mat(3, 3, CvType.CV_32FC1);
                Mat distCoeffs = new Mat(1, 5, CvType.CV_32FC1);

                cameraMatrix.put(0, 0, cameraParam[0]);
                distCoeffs.put(0, 0, cameraParam[1]);

                Imgproc.undistortPoints(corrected_topleft, topleft_undistort, cameraMatrix, distCoeffs);
                Imgproc.undistortPoints(corrected_bottomright, bottomright_undistort, cameraMatrix, distCoeffs);

                Log.d("undistorted", topleft_undistort.get(0, 0)[0] + ", " + topleft_undistort.get(0, 0)[1]);
                Log.d("undistorted", bottomright_undistort.get(0, 2)[0] + ", " + bottomright_undistort.get(0, 2)[1]);

                double[] target_coord = {
                        (((topleft_undistort.get(0, 0)[0] * cameraParam[0][0]) + cameraParam[0][2]) + ((bottomright_undistort.get(0, 2)[0] * cameraParam[0][0]) + cameraParam[0][2])) / 2,
                        (((topleft_undistort.get(0, 0)[1] * cameraParam[0][4]) + cameraParam[0][5]) + ((bottomright_undistort.get(0, 2)[1] * cameraParam[0][4]) + cameraParam[0][5])) / 2
                };
                Log.d("AR[target_1]", target_coord[0] + ", " + target_coord[1]);

                double[] target_coord_second = {
                        (((topleft_undistort.get(0, 2)[0] * cameraParam[0][0]) + cameraParam[0][2]) + ((bottomright_undistort.get(0, 0)[0] * cameraParam[0][0]) + cameraParam[0][2])) / 2,
                        (((topleft_undistort.get(0, 2)[1] * cameraParam[0][4]) + cameraParam[0][5]) + ((bottomright_undistort.get(0, 0)[1] * cameraParam[0][4]) + cameraParam[0][5])) / 2
                };
                Log.d("AR[target_2]", target_coord_second[0] + ", " + target_coord_second[1]);

                result[0] = (target_coord[0] + target_coord_second[0]) / 2;
                result[1] = (target_coord[1] + target_coord_second[1]) / 2;
                detected = true;

            } catch (Exception e) {
                Log.d("AR[status]:", " Not detected");
                Log.e("AR[status]:", "error", e);
            }
            //////////////////////////////////////////////////////////////////////////////////////////////////////
            count++;
        }

        Log.d("AR[status]:", "finish");
        long stop_time = SystemClock.elapsedRealtime();

        Log.d("AR[count]:", " " + count);
        Log.d("AR[total_time]:", " " + (stop_time - start_time) / 1000);
        return result;
    }
}

