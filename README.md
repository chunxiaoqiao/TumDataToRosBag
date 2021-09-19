# TumDataToRosBag

read TUM dataset and write to rosbag

string filePath="/media/q/neu/dataset/TUM/4season数据集/buinesspark/recording_2021-02-25_14-16-43_imu_gnss/recording_2021-02-25_14-16-43/";//imu 数据位置
string imgPath = "/media/q/neu/dataset/TUM/4season数据集/buinesspark/recording_2021-02-25_14-16-43_stereo_images_undistorted/recording_2021-02-25_14-16-43/";//图像数据位置

run 
rosrun TumDataToRosBag /media/q/neu/dataset/TUM/4season数据集/buinesspark/recording_2021-02-25_14-16-43_imu_gnss/recording_2021-02-25_14-16-43/  /media/q/neu/dataset/TUM/4season数据集/buinesspark/recording_2021-02-25_14-16-43_stereo_images_undistorted/recording_2021-02-25_14-16-43/
