import rospy
import baxter_interface


rospy.init_node('Hello_Baxter')

side = 'left'
limb = baxter_interface.Limb(side)
angles = limb.joint_angles()


#angle_list = [0.366, -0.206, -1.458, 1.665, 2.930, -1.124, -0.142] # left best
angle_list = [-0.158,0.880,-2.753,1.710,1.174,1.690,2.113] # left worst
#angle_list = [0.018,-0.849,0.516,1.797,1.191,1.604,-0.705] # right
#angle_list = [0]*7

"""
# Initial
angles[side+'_s0']=0.0947
angles[side+'_s1']=-1.3848
angles[side+'_e0']=-0.3447
angles[side+'_e1']=2.2595
angles[side+'_w0']=0.1863
angles[side+'_w1']=-0.8544
angles[side+'_w2']=-0.0747
"""

"""
# Natural Final
angles[side+'_s0']=-0.8644
angles[side+'_s1']=-0.5584
angles[side+'_e0']=-0.3937
angles[side+'_e1']=1.0704
angles[side+'_w0']=-0.2851
angles[side+'_w1']=-0.5974
angles[side+'_w2']=0.6482
"""

"""
# Robust
angles[side+'_s0']=-0.8711
angles[side+'_s1']=-1.3939
angles[side+'_e0']=0.7727
angles[side+'_e1']=2.2749
angles[side+'_w0']=0.3868
angles[side+'_w1']=-0.9350
angles[side+'_w2']=-0.4058
"""

angles[side+'_s0']=angle_list[0]
angles[side+'_s1']=angle_list[1]
angles[side+'_e0']=angle_list[2]
angles[side+'_e1']=angle_list[3]
angles[side+'_w0']=angle_list[4]
angles[side+'_w1']=angle_list[5]
angles[side+'_w2']=angle_list[6]

limb.move_to_joint_positions(angles)

