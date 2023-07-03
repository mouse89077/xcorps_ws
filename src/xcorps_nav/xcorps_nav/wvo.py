import numpy as np
from scipy.spatial import ConvexHull
import matplotlib.path as mpath

# functions which return something
def cal_CPA(OS_pos, TS_pos, OS_spd, TS_spd, OS_heading, TS_heading): # all current values, not array
# Usage: tCPA, dCPA = cal_CPA(self.OS_enu_pos[-1, :], self.TS_enu_pos[-1, :], self.OS_spd[-1], self.TS_spd[-1], self.OS_heading[-1], self.TS_heading[-1])
    
    # Convert heading values to radians
    OS_heading_rad = np.radians(OS_heading)
    TS_heading_rad = np.radians(TS_heading)
    # Calculate relative position of two vessels
    rel_pos = np.array(OS_pos) - np.array(TS_pos)
    # Calculate relative velocity of two vessels
    OS_enu_vel = [OS_spd * np.cos(OS_heading_rad), OS_spd * np.sin(OS_heading_rad)]
    TS_enu_vel = [TS_spd * np.cos(TS_heading_rad), TS_spd * np.sin(TS_heading_rad)]
    rel_vel = np.array(OS_enu_vel) - np.array(TS_enu_vel)
    
    # Calculate tCPA
    norm_rel_vel = np.linalg.norm(rel_vel)
    if norm_rel_vel != 0:
        tCPA = np.dot(rel_pos, rel_vel) / (norm_rel_vel ** 2)
    else:
        tCPA = 0.0
        
    # calculate enu position at CPA
    OS_pos_CPA = OS_pos + [value * tCPA for value in OS_enu_vel]
    TS_pos_CPA = TS_pos + [value * tCPA for value in TS_enu_vel]

    # Calculate dCPA
    dCPA = np.linalg.norm(OS_pos_CPA - TS_pos_CPA)

    return tCPA, dCPA

def cal_pre_col_ch(tCPA, dCPA, tmin, dmax):
# Usage: pre_col_ch = cal_pre_col_ch(self.tCPA[-1], self.dCPA[-1], self.tmin, self.dmax)
    if tCPA < tmin and dCPA < dmax:
        pre_col_ch = True 
    else:
        pre_col_ch = False

    return pre_col_ch

def cal_rotation(heading):
# Usage: R = cal_rotation(self.OS_heading[-1])
    heading_rad = np.radians(heading)
    rot_mat = np.array([[np.cos(heading_rad), -np.sin(heading_rad)], [np.sin(heading_rad), np.cos(heading_rad)]])
    return rot_mat

def cal_shape(pos, dim, heading):
# Usage: OS_shape = cal_shape(self.OS_enu_pos[-1, :], self.OS_dim, self.OS_heading[-1])
# Usage: TS_shape = cal_shape(self.TS_enu_pos[-1, :], self.TS_dim, self.TS_heading[-1])
    heading_rad = np.radians(heading)
    temp_x = np.transpose([2/3, 1/3, -1/3, -1/3, 1/3])
    temp_x = [value * dim[0] for value in temp_x]
    temp_y = np.transpose([0, 1/2, 1/2, -1/2, -1/2])
    temp_y = [value * dim[1] for value in temp_y]
    temp = np.hstack((temp_x, temp_y))
    R = cal_rotation(heading_rad)
    r_temp = [value * R for value in temp]
    rt_temp = [value + pos for value in r_temp]
    shape = np.array(rt_temp)
    return shape

def mink_sum(A, B):
    # A = - OS_shape
    # B = TS_shape
    # sA = size(A)
    # sB = size(B)
    # AB = reshape(A, [1, sA]) + reshape(B, [sB(1), 1, sB(2)])
    # AB = reshape(AB, [], 2)
    # mink_sum = AB + OS_pos
    
# Usage: Mink_sum = mink_sum(-OS_shape, TS_shape)
    ms = []
    for point1 in A:
        for point2 in B:
            sum_point = [point1[0] + point2[0], point1[1] + point2[1]]
            ms.append(sum_point)

    return ms

def conv_hull(mink_sum):
# Usage: Conv_hull = conv_hull(Mink_sum)
    hull = ConvexHull(mink_sum)
    hull_points = mink_sum[hull.vertices]
    return hull_points

def cal_CC(OS_pos, Conv_hull):
# Usage: CC = cal_CC(self.OS_enu_pos[-1, :], Conv_hull)

    # OS_shape = n x 2 array of [x, y]
    # TS_shape = m x 2 array of [x, y]
    # OS_pos = [x, y]
    # TS_pos = [x, y]
    # to be filled...
    grad = np.zeros(len(Conv_hull[:, 0]))
    for i in range(len(Conv_hull[:, 0])):
        grad[i] = np.arctan2(Conv_hull[i, 1]-OS_pos[1], Conv_hull[i, 0]-OS_pos[0])
    
    max_idx = np.argmax(grad)
    min_idx = np.argmin(grad)

    CC = np.zeros((3, 2))
    CC[0, :] = Conv_hull[min_idx, :]
    CC[1, :] = OS_pos
    CC[2, :] = Conv_hull[max_idx, :]
    
    return CC

def cal_VO(OS_spd, TS_spd, OS_heading, TS_heading, CC):
# Usage: VO = cal_VO(self.OS_spd[-1], self.TS_spd[-1], self.OS_heading[-1], self.TS_heading[-1], CC)
    
    # Convert heading values to radians
    OS_heading_rad = np.radians(OS_heading)
    TS_heading_rad = np.radians(TS_heading)
    # Calculate relative velocity of two vessels
    OS_enu_vel = [OS_spd * np.cos(OS_heading_rad), OS_spd * np.sin(OS_heading_rad)]
    TS_enu_vel = [TS_spd * np.cos(TS_heading_rad), TS_spd * np.sin(TS_heading_rad)]
    rel_vel = np.array(TS_enu_vel) - np.array(OS_enu_vel)
    # Calculate VO from CC
    VO = [value + rel_vel for value in CC]
    
    return VO
    
def cal_WVO(OS_spd, TS_spd, OS_heading, VO, alpha, tmin):
# Usage: WVO = cal_WVO(self.OS_spd[-1], self.TS_spd[-1], self.OS_heading[-1], VO, self.alpha, self.tmin)

    a = alpha * OS_spd * tmin / 3
    b = a / 2
    ell = cal_ellipse([0, 0], a, b, OS_heading) 
    ms = mink_sum(ell, VO)
    ch = conv_hull(ms)
    WVO = ch

    return WVO

def cal_ellipse(Center, a, b, heading):
# Used for calculating WVO
# Usage: Ellipse = cal_ellipse([Center_x, Center_y], a, b, OS_heading)

    theta = np.linspace(0, 2*np.pi, 10)
    x = np.transpose([value * a for value in theta])
    y = np.transpose([value * b for value in theta])
    ell = np.hstack(x, y)
    r_ell = [cal_rotation(heading) * value for value in ell]
    rt_ell = [value + Center for value in r_ell]
    return rt_ell  

def cal_RV(heading, spd_lim, heading_lim):
# Usage: RV = cal_RV(self.OS_heading[-1], self.spd_lim, self.heading_lim)
# RV = n x 2 array of [spd, heading_deg]
    
    spd_res = 0.1
    heading_res = 1.0 # degrees
    spd_arr = np.arange(spd_lim[0], spd_lim[1], spd_res)
    heading_arr = np.arange(heading_lim[0], heading_lim[1], heading_res)
    heading_arr = [value + heading for value in heading_arr]

    rv = []
    RV = []
    for spd in spd_arr:
        for hdg in heading_arr:
            if hdg > 180:
                hdg -= 360
            elif hdg < -180:
                hdg += 360

            rv = [spd, hdg]
            RV = np.append(RV, rv)

    RV = np.reshape(RV, (-1, 2))

    return RV

def cal_RP(OS_pos, RV):
# Usage: RP = cal_RP(self.OS_enu_pos[-1, :], RV)
    
    if len(RV) == 0:
        RP = []
    else:
        RP = np.zeros((len(RV), 2))
        RP[:, 0] = OS_pos[0] + RV[:, 0] * np.cos(RV[:, 1])
        RP[:, 1] = OS_pos[1] + RV[:, 0] * np.sin(RV[:, 1])

    return RP
    
def cal_RAV(RAV, RAP, WVO):
# Usage: RAV = cal_RAV(RAV, RAP, WVO)

    if len(RAV) == 0:
        RAV = []
    else:
        polygon_path = mpath.Path(WVO)
        test_points = RAP
        results = polygon_path.contains_points(test_points)
        RAV = test_points[results]

    return RAV

def cal_cmd_vel(OS_pos, RAV, ref_spd, waypoint, des_spd, des_heading):
# Usage: des_spd, des_heading = cal_cmd_vel(self.OS_enu_pos[-1, :], RAV, self.ref_spd, self.waypoint, self.des_spd[-1], self.des_heading[-1])

# only use this function if len(RAV) != 0

    ref_ang = np.arctan2(waypoint[1] - OS_pos[1], waypoint[0] - OS_pos[0])
    ref_ang = np.degrees(ref_ang)
    ref_vel = [ref_spd, ref_ang]
    
    RAV = np.array(RAV)  # Convert RAV to a NumPy array
    cost = [value - ref_vel for value in RAV]
    idx = np.argmin(np.linalg.norm(cost))
    des_spd = RAV[idx, 0]
    des_heading = RAV[idx, 1]
    
    return des_spd, des_heading
