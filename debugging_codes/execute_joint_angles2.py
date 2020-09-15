import rospy
import baxter_interface


rospy.init_node('Hello_Baxter')

side = 'left'
limb = baxter_interface.Limb(side)
angles = limb.joint_angles()

# angle_list = [-0.0990,   -0.8800,   -0.6390,    1.5560,    0.6180,   -0.6210,   -1.6550]

#angle_list = [-0.3630,   -0.9839,   -0.5105,    1.6870,    0.2991,   -0.6811,   -1.1799]

# angle_list = [-0.6208,   -0.9721,   -0.3901,    1.7240,    0.0656,   -0.7484,   -0.7797]

# angle_list = [-0.9126,   -0.8369,   -0.2574,    1.6659,   -0.1550,   -0.8429,   -0.3402]

# angle_list = [-1.0668,   -0.6983,   -0.1845,    1.5690,   -0.2627,   -0.8902,   -0.1008]

# angle_list = [-1.1739,   -0.5597,   -0.1303,    1.4533,   -0.3423,   -0.9125,    0.0822]

# angle_list = [-1.2539,   -0.4178,   -0.0869,    1.3190,   -0.4121,   -0.9166,    0.2444]

angle_list = [-1.2676,   -0.3882,   -0.0792,    1.2891,   -0.4259,   -0.9154,    0.2766]

angles[side+'_s0']=angle_list[0]
angles[side+'_s1']=angle_list[1]
angles[side+'_e0']=angle_list[2]
angles[side+'_e1']=angle_list[3]
angles[side+'_w0']=angle_list[4]
angles[side+'_w1']=angle_list[5]
angles[side+'_w2']=angle_list[6]

limb.move_to_joint_positions(angles)

