import sys
import os

import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Point, Quaternion

rospy.init_node("respawn")
y = 45
x = 15
path = (os.path.abspath(os.path.join(os.path.dirname( os.getcwd() ), '..', 'mp3')))

print(path)
rospy.wait_for_service("/gazebo/delete_model")
try:
    delete = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
    delete("polaris")
    delete("marker")
except rospy.ServiceException as e:
    print("Service call failed: ",e)

rospy.wait_for_service("/gazebo/spawn_urdf_model")
try:
    spawner = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
    spawner("polaris", open(path + "/urdf/polaris.urdf",'r').read(), "polaris", Pose(position= Point(x,y,1),orientation=Quaternion(0,0,0.707,0.707)),"world")
except rospy.ServiceException as e:
    print("Service call failed: ",e)


rospy.wait_for_service("/gazebo/spawn_sdf_model")
try:
    spawner = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    spawner("marker", open(path + "/models/marker/model.sdf",'r').read(), "marker", Pose(position= Point(x,y,1),orientation=Quaternion(0,0,0,0)),"world")
except rospy.ServiceException as e:
    print("Service call failed: ",e)
