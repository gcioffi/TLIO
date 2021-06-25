import os
import rosbag
from geometry_msgs.msg import Point
import pandas as pd

if __name__ == '__main__':
    bagfile = "/home/rpg/recording/mybag.bag"
    bag = rosbag.Bag(bagfile)
    topic_ts = '/timestamp'
    topic_IMU = '/IMU'
    topic_evolving_state = '/evolving_state
    topic_calib_state = '/calib_state'
    topic_attitude = '/attitude'


    column_name_ts = ['t']
    df_ts = pd.DataFrame(columns=column_name_ts)

    column_name_IMU = ['t', 'acc_raw', 'acc_cal', 'gyr_raw', 'gyr_cal', 'has_vio']
    df_IMU = pd.DataFrame(columns=column_name_IMU)

    column_name_evolving_state = ['t', 'q_wxyz', 'p', 'v']
    df_evolving_state = pd.DataFrame(columns=column_name_evolving_state)

    column_name_calib_state = ['t', 'acc_scale_inv', 'gyr_scale_inv', 'gyro_g_sense', 'b_acc', 'b_gyr']
    df_calib_state = pd.DataFrame(columns=column_name_calib_state)

    column_name_atittude = ['t', 'qw', 'qx', 'qy', 'qz']
    df_attitude = pd.DataFrame(columns=column_name_atittude)


    for topic, msg, t in bag.read_messages([topic_ts, topic_IMU, topic_evolving_state, topic_calib_state, topic_attitude]):
        if topic == topic_ts:
            ts_msg = msg.time #or head
            df_ts = df_ts.append(
                {'t' : ts_msg},
                ignore_index=True
            )

        if topic == topic_IMU:
            ts_msg = msg.time
            acc_raw_msg = [msg.acc_raw.x msg.acc_raw.y msg.acc_raw.z]
            acc_cal_msg = [msg.acc_cal.x msg.acc_cal.y msg.acc_cal.z]
            gyr_raw_msg = [msg.gyr_raw.x msg.gyr_raw.y msg.gyr_raw.z]
            gyr_cal_msg = [msg.gyr_cal.x msg.gyr_cal.y msg.gyr_cal.z]
            #has_vio_msg: how do I obtain this?
            df_IMU = df_IMU.append(
                {'t' : ts_msg,
                 'acc_raw' : acc_raw_msg,
                 'acc_cal' : acc_cal_msg,
                 'gyr_raw' : gyr_raw_msg,
                 'gyr_cal' : gyr_cal_msg,
                 #'has_vio' : how do I obtain this?
                },
                ignore_index=True
            )


        if topic == topic_evolving_state:
            ts_msg = msg.time
            q_wxyz_msg = [msg.q_wxyz_I msg.q_wxyz_II msg.q_wxyz_III msg.q_wxyz_IV]
            p_msg = [msg.p_x msg.p_y msg.p_z]
            v_msg = [msg.v_x msg.v_y msg.v_z]
            df_evolving_state = df_evolving_state.append(
                {'t' : ts_msg,
                 'q_wxyz' : q_wxyz_msg,
                 'p' : p_msg,
                 'v' : v_msg,
                },
                ignore_index=True
            )


        if topic == topic_calib_state:
            ts_msg = msg.time
            acc_scale_inv_msg = [msg.scale_inv.I msg.scale_inv.II msg.scale_inv.III msg.scale_inv.IV msg.scale_inv.V msg.scale_inv.VI msg.scale_inv.VII msg.scale_inv.VIII msg.scale_inv.IX]
            gyr_scale_inv_msg = [msg.gyr_scale_inv.I msg.gyr_scale_inv.II msg.gyr_scale_inv.III msg.gyr_scale_inv.IV msg.gyr_scale_inv.V msg.gyr_scale_inv.VI msg.gyr_scale_inv.vII msg.gyr_scale_inv.VIII msg.gyr_scale_inv.IX]
            gyro_g_sense_msg = [msg.gyro_g_sense.I msg.gyro_g_sense.II msg.gyro_g_sense.III msg.gyro_g_sense.IV msg.gyro_g_sense.V msg.gyro_g_sense.VI msg.gyro_g_sense.VII msg.gyro_g_sense.VIII msg.gyro_g_sense.IX]
            b_acc_msg = [msg.b_acc.I msg.b_acc.II msg.b_acc.III]
            b_gyr_msg = [msg.b_gyr.I msg.b_gyr.II msg.b_gyr.III]
            df_calib_state = df_calib_state.append(
                {'t' : ts_msg,
                 'acc_scale_inv' : acc_scale_inv_msg,
                 'gyr_scale_inv' : gyr_scale_inv_msg,
                 'gyro_g_sense' : gyro_g_sense_msg,
                 'b_acc' : b_acc_msg,
                 'b_gyr' : b_gyr_msg,
                },
                ignore_index=True
            )


        if topic == topic_attitude:
            ts_msg = msg.time
            q_w_msg = [msg.q_w]
            q_x_msg = [msg.q_x]
            q_y_msg = [msg.q_y]
            q_z_msg = [msg.q_z]
            df_evolving_state = df_evolving_state.append(
                {'t' : ts_msg,
                 'q_wxyz' : q_wxyz_msg,
                 'p' : p_msg,
                 'v' : v_msg,
                },
                ignore_index=True
            )




    df_ts.to_csv('ts.csv')
    df_ts.to_csv(r'c:\data\ts.txt', header=None, index=None, sep='\t', mode='a')

    df_IMU.to_csv('IMU.csv')
    df_IMU.to_csv(r'c:\data\IMU.txt', header=None, index=None, sep='\t', mode='a')

    df_evolving_state.to_csv('evolving_state.csv')
    df_evolving_state.to_csv(r'c:\data\evolving_state.txt', header=None, index=None, sep='\t', mode='a')
 
    df_calib_state.to_csv('calib_state.csv')
    df_calib_state.to_csv(r'c:\data\calib_state.txt', header=None, index=None, sep='\t', mode='a')

    df_attitude.to_csv('attitude.csv')
    df_attitude.to_csv(r'c:\data\attitude.txt', header=None, index=None, sep='\t', mode='a')
    
    bag.close()