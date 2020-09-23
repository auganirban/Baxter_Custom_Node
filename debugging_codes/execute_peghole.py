import rospy
import baxter_interface

# Best
best_left = [-0.362,0.321,-2.994,0.572,1.279,1.932,-0.494]
best_right = [0.494,0.551,2.881,1.210,-1.367,1.552,0.840]

# Worst
worst_left = [-0.120,0.084,-1.980,0.507,0.324,1.810,-0.347]
worst_right = [0.278,-0.710,0.710,1.203,-2.090,-1.336,3.050]

# worst_left = [0]*7
# worst_right = [0]*7

def move_side_limb(side,solType):
  limb = baxter_interface.Limb(side)
  angles = limb.joint_angles()
  
  if (side == "left"):
    if (solType == "best"):
      angle_list = best_left
    else:
      angle_list = worst_left
  else:
    if (solType == "best"):
      angle_list = best_right
    else:
      angle_list = worst_right
      
  angles[side+'_s0'] = angle_list[0]
  angles[side+'_s1'] = angle_list[1]
  angles[side+'_e0'] = angle_list[2]
  angles[side+'_e1'] = angle_list[3]
  angles[side+'_w0'] = angle_list[4]
  angles[side+'_w1'] = angle_list[5]
  angles[side+'_w2'] = angle_list[6]
  
  limb.move_to_joint_positions(angles)
  
  
if __name__ == "__main__":
  rospy.init_node('DualArmPose')
  move_side_limb("left","worst")
  move_side_limb("right","worst")
  print("Execution completed")

