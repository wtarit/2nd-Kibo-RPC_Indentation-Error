private double[] readQR(int count_max) {
        String contents = null;
        int count = 0;
        double pattern = 0, final_x = 0, final_y = 0, final_z = 0;

        long start_time = SystemClock.elapsedRealtime();
        while (contents == null && count < count_max) {
            Log.d("DevLog_status_QR[status]:", " start");

            try {
                Mat img = new Mat(api.getMatNavCam(), new Rect(600, 600, 200, 200));

                long start_time_preproc = SystemClock.elapsedRealtime();
                Mat binary_img = thresholding(img);
                List<MatOfPoint> contours = new ArrayList<>();
                Mat hierarchy = new Mat();
                Imgproc.findContours(binary_img, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

                Rect target_rect = new Rect();
                for (int i = 0; i < contours.size(); i++) {
                    if (hierarchy.get(0, i)[2] == -1.0) {
                        MatOfPoint2f ct2f = new MatOfPoint2f(contours.get(i).toArray());
                        MatOfPoint2f approxCurve = new MatOfPoint2f();

                        double approxDistance = Imgproc.arcLength(ct2f, true) * 0.1;
                        Imgproc.approxPolyDP(ct2f, approxCurve, approxDistance, true);
                        MatOfPoint points = new MatOfPoint(approxCurve.toArray());

                        if (points.size().height == 4.0) {
                            target_rect = Imgproc.boundingRect(points);
                        }
                    }
                }
                long stop_time_preproc = SystemClock.elapsedRealtime();
                Log.d("QR[preprocess_time]:", " " + (stop_time_preproc - start_time_preproc));

                Log.d("QR[target_rect]", target_rect.toString());
                img = new Mat(img, target_rect);
                byte[] pixels_byte = new byte[target_rect.width * target_rect.height];
                img.get(0, 0, pixels_byte);

                Image barcode = new Image(target_rect.width, target_rect.height, "Y800");
                barcode.setData(pixels_byte);

                ImageScanner reader = new ImageScanner();
                reader.setConfig(Symbol.NONE, Config.ENABLE, 0);
                reader.setConfig(Symbol.QRCODE, Config.ENABLE, 1);

                int result = reader.scanImage(barcode);

                if (result != 0) {
                    SymbolSet syms = reader.getResults();
                    for (Symbol sym : syms) {
                        contents = sym.getData();
                    }
                }
                Log.d("DevLog_status_QR[raw data]:", contents);
                Log.d("DevLog_status_QR[status]:", "Done reading");

                String[] multi_contents = contents.split(",");
                pattern = Double.parseDouble(multi_contents[0].split(":")[1]);
                final_x = Double.parseDouble(multi_contents[1].split(":")[1]);
                final_y = Double.parseDouble(multi_contents[2].split(":")[1]);
                String tmp = multi_contents[3].split(":")[1];
                final_z = Double.parseDouble(tmp.substring(0, tmp.length() - 1));
                api.sendDiscoveredQR(contents);
            } catch (Exception e) {
                Log.e("DevLog_status_QR[status]:", " Not detected", e);
            }
            //////////////////////////////////////////////////////////////////////////////////////////////////////
            count++;
        }
        Log.d("DevLog_status_QR[status]:", " stop");
        long stop_time = SystemClock.elapsedRealtime();

        Log.d("DevLog_status_QR[count]:", " " + count);
        Log.d("DevLog_status_QR[total_time]:", " " + (stop_time - start_time));

        return new double[]{pattern, final_x, final_y, final_z};
    }