import rosbag

print("Ok!")
bag = rosbag.Bag('traj2.bag')
for topic, msg, t in bag.read_messages(topics=['/vicon/parrot']):
    #print(msg)
    msg.header.frame_id = "world"
bag.close()
