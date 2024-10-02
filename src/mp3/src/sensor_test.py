from lidarProcessing import LidarProcessing
import rospy

def visualize_sensor():
    side_range = (-20, 20)
    fwd_range = (-20., 20.)
    height_range = (-1.5, 0.5)
    resolution = 0.1
    rate = rospy.Rate(100)  # 100 Hz

    lidar = LidarProcessing(resolution=resolution, side_range=side_range, fwd_range=fwd_range,
                         height_range=height_range)    
    while True:
        lidar.processLidar()
        rate.sleep()
        pass

if __name__ == "__main__":
    rospy.init_node('sensor_test')    
    visualize_sensor()