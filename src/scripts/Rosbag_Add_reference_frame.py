import rosbag

bag = rosbag.Bag('traj2.bag')
for topic, msg, t in bag.read_messages(topics=['/vicon/parrot']):
    msg.header.frame_id = "world"
bag.close()
