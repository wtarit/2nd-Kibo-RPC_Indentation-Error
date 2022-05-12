package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.os.Environment;
import android.os.SystemClock;
import android.util.Log;

import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;

import java.io.File;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {
    @Override
    protected void runPlan1(){
        api.startMission();

        //Point A
        double[] robot_pos = {11.21, -9.8, 5.014};
        Quaternion face_airlock = new Quaternion(0f, 0f, -0.707f, 0.707f);
        moveToWrapper(robot_pos, face_airlock);
        SystemClock.sleep(1000);
        captureImage(api.getMatNavCam(), "pointa.png");
    }

    @Override
    protected void runPlan2(){
        // write here your plan 2
    }

    @Override
    protected void runPlan3(){
        // write here your plan 3
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
        Log.d("Result[getMessage]", result.getMessage());
        Log.d("Result[tostring]", result.toString());
        Log.d("Result[status]", "" + result.getStatus().getValue());

        return true;
    }

    public void captureImage(Mat mInter, String filename) {
        File path = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_PICTURES);
        File file = new File(path, filename);
        Boolean bool = null;
        filename = file.toString();

        bool = Imgcodecs.imwrite(filename, mInter);
        if (bool == true)
            Log.d("imwrite", "SUCCESS writing image to external storage");
        else
            Log.d("imwrite", "Fail writing image to external storage");
    }

}

