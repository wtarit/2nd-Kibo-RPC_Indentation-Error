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