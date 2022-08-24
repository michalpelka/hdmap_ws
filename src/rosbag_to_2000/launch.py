import roslaunch
import rospy
import rospkg
import subprocess
import os
import rosparam


bag_dir = "/media/michal/ext/orto_photo_skierniewice/hd_skierniewice/2022-08-19T13:05:12.524438/"
rate = 5

output_bag = os.path.join(bag_dir, "processed_lio.bag")
print ("Output bag ", output_bag)
# sim time
rosparam.set_param('use_sim_time', "true")

# list bag files
bag_dir_files = []
for file in os.listdir(bag_dir):
    if file.endswith(".bag") and file.startswith("log_"):
        bag_dir_files.append(os.path.join(bag_dir, file))
for bag_file in bag_dir_files:
    print("bag_file ", bag_file)

# start fast-lio
rospy.init_node('en_Mapping', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
rospack = rospkg.RosPack()
fast_lio_path = rospack.get_path('fast_lio')
launch = roslaunch.parent.ROSLaunchParent(uuid, [fast_lio_path+"/launch/mapping_horizon.launch"])
launch.start()
rospy.loginfo("started")


# start playback
params = ["rosbag", "play", "--clock", "--rate", "5"]
for bag_file in bag_dir_files:
    params.append(bag_file)
bag_play = subprocess.Popen(params)

bag_record = subprocess.Popen(['rosbag', 'record', '-O',output_bag, "/Odometry"])

# wait to end
bag_play.wait()
bag_record.terminate()
launch.shutdown()