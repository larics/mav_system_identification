function pointcloud = readPointcloud(bag, topic)
%READPOINTCLOUD Reads given pointcloud message from the bag object

velodyne_raw = bag.readAll(topic);
size = length(velodyne_raw);

imu.

end

