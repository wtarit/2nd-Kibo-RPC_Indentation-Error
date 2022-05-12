package jp.jaxa.iss.kibo.rpc.thailand;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import android.os.SystemClock;
import android.util.Log;
// android library

import net.sourceforge.zbar.Config;
import net.sourceforge.zbar.Image;
import net.sourceforge.zbar.ImageScanner;
import net.sourceforge.zbar.Symbol;
import net.sourceforge.zbar.SymbolSet;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {

    @Override
    protected void runPlan1() {
        // astrobee is undocked and the mission starts
        api.startMission();

        double[] robot_pos = {11.21, -9.8, 5.012};
        Quaternion face_airlock = new Quaternion(0f, 0f, -0.707f, 0.707f);
        moveToWrapper(robot_pos, face_airlock);
        Log.d("DevLog_status", "moved to point a");

//        SystemClock.sleep(3000);
        readQR();

        //align robot with target
        //{yaw, pitch, roll}
        double[] current_orientation = {-90, 0, 0};
        double[] laser_point = {0.1302, 0.1111};
        double pivot_laser_target_angle = 139.525787;
        double r1 = 0.1711585522;  //pivot_to_laser_length
        double[] target_point = new double[2];

        double[] target_pos = calTargetPos();
        target_point[0] = 0.785;
//        target_point[1] = target_pos[1] + 0.0826;
        target_point[1] = target_pos[1] + 0.0926;

        Log.d("AR[target_point(xy plane)]", target_point[0] + ", " + target_point[1]);

        // square root(delta x square + delta y square)
        double pivot_to_target_length = Math.sqrt(Math.pow(target_point[0], 2) + Math.pow(target_point[1], 2));
        Log.d("AR[pivot_to_target_length]", "" + pivot_to_target_length);

        double a = 1;
        double b = 2 * r1 * Math.cos(Math.toRadians(180 - pivot_laser_target_angle));
        double c = Math.pow(r1, 2) - Math.pow(pivot_to_target_length, 2);
        double r2 = (-b + Math.sqrt(Math.pow(b, 2) - 4 * a * c)) / 2 * a;

        Log.d("AR[radius]", r1 + ", " + r2);

        double[] laser_shooting_coord = find_laser_point(target_point, r1, r2);
        double laser_origin_to_shooting_length = Math.sqrt(Math.pow(laser_point[0] - laser_shooting_coord[0], 2) + Math.pow(laser_point[1] - laser_shooting_coord[1], 2));

        Log.d("AR[laser_shooting_coord]", laser_shooting_coord[0] + ", " + laser_shooting_coord[1]);
        Log.d("AR[laser_origin_to_shooting_length]", "" + laser_origin_to_shooting_length);

        double pitch = 2 * Math.toDegrees(Math.asin((0.5 * laser_origin_to_shooting_length) / r1));
        Log.d("AR[pitch]", "" + pitch);
        if (target_point[1] != 0 && pitch < 45) {
            current_orientation[1] = current_orientation[1] - pitch;
        } else {
            //Just to be safe
            current_orientation[1] = current_orientation[1] - 30;
        }

//        robot_pos[0] = robot_pos[0] + (target_pos[0] - 0.0994);
//        robot_pos[0] = robot_pos[0] + (target_pos[0] - 0.092);
        robot_pos[0] = robot_pos[0] + (target_pos[0] - 0.1094);
        Log.d("AR[Robot Position]", robot_pos[0] + ", " + current_orientation[1]);
        Quaternion q = eulerToQuaternion(current_orientation[0], current_orientation[1], current_orientation[2]);
        moveToWrapper(robot_pos, q);

        // irradiate the laser and take snapshots
        api.laserControl(true);
        api.takeSnapshot();
        api.laserControl(false);

        ///////// move to point B //////////
        Log.d("DevLog_status", "Start moving to point B");
        moveToWrapper(10.67, -8.76, 4.7227, face_airlock);
        moveToWrapper(10.6, -8.0, 4.5, face_airlock);

        Log.d("Report", "Reached point B Start report mission");
        api.reportMissionCompletion();
        Log.d("Report", "report mission complete");
    }

    @Override
    protected void runPlan2() {
        // write here your plan 2
    }

    @Override
    protected void runPlan3() {
        // write here your plan 3
    }

    private double[] find_laser_point(double[] target_point, double r1, double r2) {
        double x1 = 0, y1 = 0, x2 = target_point[0], y2 = target_point[1];
        double centerdx = x1 - x2;
        double centerdy = y1 - y2;
        double R = Math.sqrt(centerdx * centerdx + centerdy * centerdy);
        double R2 = R * R;
        double R4 = R2 * R2;
        double a = (r1 * r1 - r2 * r2) / (2 * R2);
        double r2r2 = (r1 * r1 - r2 * r2);
        double c = Math.sqrt(2 * (r1 * r1 + r2 * r2) / R2 - (r2r2 * r2r2) / R4 - 1);
        double fx = (x1 + x2) / 2 + a * (x2 - x1);
        double gx = c * (y2 - y1) / 2;
        double ix1 = fx + gx;
        double ix2 = fx - gx;
        double fy = (y1 + y2) / 2 + a * (y2 - y1);
        double gy = c * (x1 - x2) / 2;
        double iy1 = fy + gy;
        double iy2 = fy - gy;
        if (iy1 > iy2) {
            return new double[]{ix1, iy1};
        }
        return new double[]{ix2, iy2};
    }

    private boolean moveToWrapper(double pos_x, double pos_y, double pos_z, Quaternion quaternion) {

        final Point point = new Point(pos_x, pos_y, pos_z);

        Result result = api.moveTo(point, quaternion, true);

        int loopCounter = 0;
        while (!result.hasSucceeded() && loopCounter < 3) {
            result = api.moveTo(point, quaternion, true);
            loopCounter++;
        }

        Log.d("move[count]", "" + loopCounter);
        return true;
    }

    private boolean moveToWrapper(double[] pos, Quaternion quaternion) {

        final Point point = new Point(pos[0], pos[1], pos[2]);

        Result result = api.moveTo(point, quaternion, true);

        int loopCounter = 0;
        while (!result.hasSucceeded() && loopCounter < 3) {
            result = api.moveTo(point, quaternion, true);
            loopCounter++;
        }
        Log.d("move[count]", "" + loopCounter);
        return true;
    }

    private Quaternion eulerToQuaternion(double yaw_degree, double pitch_degree, double roll_degree) {
        double yaw = Math.toRadians(yaw_degree); //radian = degree*PI/180
        double pitch = Math.toRadians(pitch_degree);
        double roll = Math.toRadians(roll_degree);

        double cy = Math.cos(yaw * 0.5);
        double sy = Math.sin(yaw * 0.5);
        double cp = Math.cos(pitch * 0.5);
        double sp = Math.sin(pitch * 0.5);
        double cr = Math.cos(roll * 0.5);
        double sr = Math.sin(roll * 0.5);

        double qx = sr * cp * cy - cr * sp * sy;
        double qy = cr * sp * cy + sr * cp * sy;
        double qz = cr * cp * sy - sr * sp * cy;
        double qw = cr * cp * cy + sr * sp * sy;

        return new Quaternion((float) qx, (float) qy, (float) qz, (float) qw);
    }

    private double[] calTargetPos() {
        long start_time = SystemClock.elapsedRealtime();
        double[][] cameraParam = api.getNavCamIntrinsics();
        Mat cameraMatrix = new Mat(3, 3, CvType.CV_32FC1);
        Mat dstMatrix = new Mat(1, 5, CvType.CV_32FC1);
        cameraMatrix.put(0, 0, cameraParam[0]);
        dstMatrix.put(0, 0, cameraParam[1]);

        Mat ids = new Mat();
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        ArrayList<Mat> corners = new ArrayList<>();
        double xdiff = 0, ydiff = 0;

        Log.d("AR[status]:", "start");
        byte ar_count = 0;
        Rect ar_roi;
        while (ids.size().height != 4 && ar_count < 3) {
            try {
                if (ar_count == 0) {
                    ar_roi = new Rect(470, 680, 360, 280);
                }
                //fall back if the aruco tag is outside of cropped boundary.
                else {
                    ar_roi = new Rect(100, 660, 1080, 300);
                }
                Mat source = new Mat(api.getMatNavCam(), ar_roi);
                Aruco.detectMarkers(source, dictionary, corners, ids);
                for (int i = 0; i < corners.size(); i++) {
                    //Get shifted aruco tag corners
                    Mat corrected_corner = corners.get(i);
                    //Shift it by the position that it get cropped.
                    corrected_corner.put(0, 0, corrected_corner.get(0, 0)[0] + ar_roi.x, corrected_corner.get(0, 0)[1] + ar_roi.y);
                    corrected_corner.put(0, 1, corrected_corner.get(0, 1)[0] + ar_roi.x, corrected_corner.get(0, 1)[1] + ar_roi.y);
                    corrected_corner.put(0, 2, corrected_corner.get(0, 2)[0] + ar_roi.x, corrected_corner.get(0, 2)[1] + ar_roi.y);
                    corrected_corner.put(0, 3, corrected_corner.get(0, 3)[0] + ar_roi.x, corrected_corner.get(0, 3)[1] + ar_roi.y);
                    //tmp mat to store undistorted corners.
                    Mat tmp = new Mat(1, 4, CvType.CV_32FC2);
                    //undistort the corners.
                    Imgproc.undistortPoints(corners.get(i), tmp, cameraMatrix, dstMatrix, new Mat(), cameraMatrix);
                    //put it back in to the same array list.
                    corners.set(i, tmp);
                }
            } catch (Exception e) {
                Log.e("AR[status]:", "error", e);
            }
            ar_count++;
        }
        if (ids.size().height == 4) {
            float markerSize = 0.05f;
            double avg_ar_size = 0;
            double tx_undistort = 0, ty_undistort = 0;
            for (Mat corner : corners) {
                double _x = 0;
                double _y = 0;
                for (int j = 0; j < corner.size().width; j++) {
                    _x = _x + corner.get(0, j)[0];
                    _y = _y + corner.get(0, j)[1];
                }
                avg_ar_size += Math.abs(corner.get(0, 0)[0] - corner.get(0, 1)[0]);
                avg_ar_size += Math.abs(corner.get(0, 2)[0] - corner.get(0, 3)[0]);
                avg_ar_size += Math.abs(corner.get(0, 0)[1] - corner.get(0, 3)[1]);
                avg_ar_size += Math.abs(corner.get(0, 1)[1] - corner.get(0, 2)[1]);
                tx_undistort += _x / 4.0;
                ty_undistort += _y / 4.0;
            }
            tx_undistort /= 4;
            ty_undistort /= 4;
            avg_ar_size /= 16;
            double pixelPerM = avg_ar_size / markerSize;
            Log.d("AR[pixelperM]", "" + pixelPerM);
            //find diff from the center of the image
            xdiff = (tx_undistort - 640) / pixelPerM;
            ydiff = (480 - ty_undistort) / pixelPerM;
        }
        long stop_time = SystemClock.elapsedRealtime();
        Log.d("AR[count]", "" + ar_count);
        Log.d("AR[total_time]:", " " + (stop_time - start_time));
        Log.d("AR[target_pos]", xdiff + ", " + ydiff);
        return new double[]{xdiff, ydiff};
    }

    private void readQR() {
        String contents = null;
        byte count = 0;

        long start_time = SystemClock.elapsedRealtime();
        byte[] pixels_byte = new byte[250 * 250];
        Image barcode = new Image(250, 250, "Y800");

        ImageScanner reader = new ImageScanner();
        reader.setConfig(Symbol.NONE, Config.ENABLE, 0);
        reader.setConfig(Symbol.QRCODE, Config.ENABLE, 1);

        while (contents == null && count < 6) {
            SystemClock.sleep(1500);
            Mat img = new Mat(api.getMatNavCam(), new Rect(595, 420, 250, 250));
            img.get(0, 0, pixels_byte);
            barcode.setData(pixels_byte);
            try {
                int result = reader.scanImage(barcode);
                if (result != 0) {
                    SymbolSet syms = reader.getResults();
                    for (Symbol sym : syms) {
                        contents = sym.getData();
                    }
                }
                Log.d("DevLog_status_QR[raw data]:", contents);
                api.sendDiscoveredQR(contents);
            } catch (Exception e) {
                Log.e("DevLog_status_QR[status]:", " Not detected", e);
            }
            count++;
        }
        while (contents == null && count < 8) {
            SystemClock.sleep(1500);
            Mat img = new Mat(api.getMatNavCam(), new Rect(250, 300, 900, 500));
            pixels_byte = new byte[900 * 500];
            barcode = new Image(900, 500, "Y800");
            img.get(0, 0, pixels_byte);
            barcode.setData(pixels_byte);
            try {
                int result = reader.scanImage(barcode);
                if (result != 0) {
                    SymbolSet syms = reader.getResults();
                    for (Symbol sym : syms) {
                        contents = sym.getData();
                    }
                }
                Log.d("DevLog_status_QR[raw data]:", contents);
                api.sendDiscoveredQR(contents);
            } catch (Exception e) {
                Log.e("DevLog_status_QR[status]:", " Not detected", e);
            }
            count++;
        }
        long stop_time = SystemClock.elapsedRealtime();

        Log.d("QR[count]:", " " + count);
        Log.d("QR[total_time]:", " " + (stop_time - start_time));
    }

}