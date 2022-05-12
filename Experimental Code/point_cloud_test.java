//Testing Hazcam
PointCloud hazcam = api.getPointCloudHazCam();
Log.d("Pointcloud", "Width:"+ hazcam.getWidth() + " Height:"+ hazcam.getHeight());
Point[] depth = hazcam.getPointArray();
Log.d("PointCloud[size]", ""+ depth.length);
Log.d("PointCloud[1]", depth[0].getX() + ", " + depth[0].getY() + ", " + depth[0].getZ());
Log.d("PointCloud[2]", depth[224].getX() + ", " + depth[224].getY() + ", " + depth[224].getZ());
