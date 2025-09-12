
import math
import numpy as np

##################################
#       Robot description        #
#
# Thigh length : 45.5 + 49 = 94.5
# Foot length : 97.64                       
# (94.529 mm, -24.467 mm)
#
# Offset foot angle : 14.51
#
# Find : Hip, Knee, Foot
#


def clamp(x, a=-1.0, b=1.0):
    return max(a, min(b, x))


def ik_leg(target, leg_base=(0.0,0.0,0.0), coxa=45.5, tl=49, fl=97.64, offset_foot_angle=0.2443461):
    # target : (x,y,z) in body frame (mm)
    # leg_base : (bx,by,bz) offset of leg base in body frame (mm)

    local_target = np.array(target) - np.array(leg_base)
    x, y, z = local_target.tolist()

    # Projection for knee and foot
    horiz = math.hypot(x, y)
    ex = horiz - coxa
    ez = z
    l = math.hypot(ex, ez)
    if l > (tl+fl) or l < abs(tl-fl) :
        print("Impossible to go here")
        return

    # Hip
    hip = (math.pi/2) - math.atan2(y, x) 

    # Foot
    foot = math.acos(clamp((tl**2 + fl**2 - l**2) / (2.0 * tl * fl))) - math.pi + offset_foot_angle

    # Knee
    va = - math.atan2(ez, ex)
    vb = math.acos(clamp((l**2 + tl**2 - fl**2) / (2.0 * l * tl)))
    knee = va - vb

    return np.array([hip, knee, foot]) 




