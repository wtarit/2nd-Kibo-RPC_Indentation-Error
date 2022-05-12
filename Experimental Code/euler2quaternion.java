package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.graphics.MaskFilter;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import java.lang.Math;

//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.CvType;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {

    //static{ System.loadLibrary(Core.NATIVE_LIBRARY_NAME); }

    @Override
    protected void runPlan1(){


        // astrobee is undocked and the mission starts
        api.startMission();

        // Move astrobee to point-A
        moveToWrapper(11.21, -9.8, 4.79, 0, 0, 0, 1);

        // irradiate the laser
        api.laserControl(true);

        // take snapshots
//        api.takeSnapshot();

        Quaternion q = eulerToQuaternion(-90,0,0);

        moveToWrapper(10.9, -9.9, 5.25, q.getX(), q.getY(), q.getZ(), q.getW());

        // Send mission completion
        api.reportMissionCompletion();
    }

    @Override
    protected void runPlan2(){
        // write here your plan 2
    }

    @Override
    protected void runPlan3(){
        // write here your plan 3
    }



    private Quaternion eulerToQuaternion(double yaw_degree, double pitch_degree, double roll_degree){

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

        Quaternion quaternion = new Quaternion((float)qx, (float)qy, (float)qz, (float)qw);

        return quaternion;
    }

//    private Mat eulerToRotation(double yaw_d, double pitch_d, double roll_d){
//        double yaw = Math.toRadians(yaw_d);
//        double pitch = Math.toRadians(pitch_d);
//        double roll = Math.toRadians(roll_d);
//        Mat Rz = new Mat(3,3,CvType.CV_64F);
//        Rz.put(0,0,
//                Math.cos(yaw),  -Math.sin(yaw), 0,
//                       Math.sin(yaw),   Math.cos(yaw), 0,
//                                   0,               0, 1);
//
//        Mat Ry = new Mat(3,3,CvType.CV_64F);
//        Ry.put(0,0,
//                Math.cos(pitch),0, Math.sin(pitch),
//                                     0,1,               0,
//                      -Math.sin(pitch),0,  Math.cos(pitch));
//
//        Mat Rx = new Mat(3,3,CvType.CV_64F);
//        Rx.put(0,0,
//                 1,              0,               0,
//                        0, Math.cos(roll), -Math.sin(roll),
//                        0, Math.sin(roll),  Math.cos(roll));
//
//        return Rz.mul(Ry.mul(Rx));
//    }

    // You can add your method
    private void moveToWrapper(double pos_x, double pos_y, double pos_z,
                               double qua_x, double qua_y, double qua_z,
                               double qua_w){

        final int LOOP_MAX = 3;
        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float)qua_x, (float)qua_y,
                                                     (float)qua_z, (float)qua_w);

        Result result = api.moveTo(point, quaternion, true);

        int loopCounter = 0;
        while(!result.hasSucceeded() || loopCounter < LOOP_MAX){
            result = api.moveTo(point, quaternion, true);
            ++loopCounter;
        }
    }

}

