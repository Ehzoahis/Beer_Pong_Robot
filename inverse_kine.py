import modern_robotics as mr
import forward_kine as fk
import numpy as np

def rot_mat(alpha, beta, gamma):
    #Rotation Matrices by x-y-z
    alpha, beta, gamma = np.radians(alpha), np.radians(beta), np.radians(gamma)
    X_Rot = np.array([[1,0,0],[0,np.cos(alpha),-np.sin(alpha)],[0,np.sin(alpha),np.cos(alpha)]])
    Y_Rot = np.array([[np.cos(beta),0,np.sin(beta)],[0,1,0],[-np.sin(beta),0,np.cos(beta)]])
    Z_Rot = np.array([[np.cos(gamma),-np.sin(gamma),0],[np.sin(gamma),np.cos(gamma),0],[0,0,1]])


    return np.dot(X_Rot, Y_Rot, Z_Rot)



def get_ik_thetas(S, M, p, init_thetas, alpha=None, beta=None, gamma=None):

    R = rot_mat(175.48, -1.59, 0)
    Xe = mr.RpToTrans(R, p)
    Xs = M

    print(T)
    #print(M)
    #print(S)

    
    eomg = 10**-2
    ev = 10**-2
    thetalist0 = np.zeros(6)
    success = 0
    while not success:
        [thetalist, success] = mr.IKinSpace(S, M, T, thetalist0, eomg, ev)
        thetalist0 = np.random.randn(6) % np.pi

    return thetalist % (2*np.pi), success






    
    #R = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
    #T = mr.RpToTrans(R, p)
    # alpha = 0
    # beta = 0
    # gamma = 0
    # success = 0
    # counter = 0
    # granularity = 2 * np.pi * 10**-2

    # thetalist0 = np.zeros(6)
    # eomg = 10**-2
    # ev = 10**-2
    
    # while not success:
    #     if counter > 100:
    #         print("Fail")
    #         return False
    #     R = np.array([[np.cos(gamma),-np.sin(gamma),0],[-np.sin(gamma),-np.cos(gamma),0],[0,0,1]])
    #     T = mr.RpToTrans(R, p)
    #     [thetalist, success] = mr.IKinSpace(S, M, T, thetalist0, eomg, ev)
    #     if success:
    #         return thetalist, success
    #     counter += 1
    #     gamma += granularity


        
    # if alpha is None:
    #     alphas = [alpha]
    # else:
    #     alphas = np.linspace(0, 2*np.pi, 100)
    # if beta is None:    
    #     betas = [beta]
    # else:
    #     betas = np.linspace(0, 2*np.pi, 100)
    # if gamma is None:
    #     gamma = [gamma]
    # else:
    #     gammas = np.linspace(0, 2*np.pi, 100)

    # for alpha in alphas:
    #     for beta in betas:
    #         for gamma in gammas:

    #             # if counter > 100:
    #             #     print("No Solution")
    #             #     break
    #             R = rot_mat(alpha,beta,gamma)
    #             # R = np.array([[np.cos(gamma),-np.sin(gamma),0],[np.sin(gamma),np.cos(gamma),0],[0,0,1]])
    #             T = mr.RpToTrans(R, p)
    #             [thetalist, success] = mr.IKinSpace(S, M, T, thetalist0, eomg, ev)
    #             if success:
    #                 return thetalist % (2*np.pi), success
    #             # thetalist0 += granularity
    #             # alpha += granularity
    #             # beta += granularity
    #             # gamma += granularity
    #             # counter += 1
    # return success
    # # return thetalist % (2*np.pi), success

def get_traj(X_s, X_e):
    return mr.ScrewTrajectory(X_s, X_e, 5, 3, 3)

def get_traj_joint(theta_s, theta_e):
    return mr.JointTrajectory(theta_s, theta_e, 5, 3, 3)

def invk(xgrip, ygrip, zgrip):
    PI = np.pi
    # theta1 to theta6
    thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    l01 = 0.152
    l02 = 0.120
    l03 = 0.244
    l04 = 0.093
    l05 = 0.213
    l06 = 0.083
    l07 = 0.083
    l08 = 0.082    
    l10 = 0.112   # thickness of aluminum plate is around 0.006

    xcen = xgrip
    ycen = ygrip
    zcen = zgrip

    # theta1
    l_xy_cen = np.sqrt(xcen**2 + ycen**2)
    b_xy_cen = 0.027 + l06
    small_angle = np.arcsin(b_xy_cen/l_xy_cen)
    large_angle = np.arctan2(ycen, xcen)
    thetas[0] = large_angle - small_angle - PI/2

    # theta6
    thetas[5] = PI/2 + thetas[0]

    # x3end, y3end in center frame
    x3end_in_cen = -l07
    y3end_in_cen = -l06 - 0.027
    z3end_in_cen = l10 + l08
    xyz3_in_cen = np.array([[x3end_in_cen, y3end_in_cen, z3end_in_cen, 1]]).T
    # translate from center frame to base frame
    T_bc = np.array([[np.cos(thetas[0]), -np.sin(thetas[0]), 0, xcen], [np.sin(thetas[0]), np.cos(thetas[0]), 0, ycen], [0, 0, 1, zcen], [0, 0, 0, 1]])
    xyz3_in_base = np.dot(T_bc, xyz3_in_cen).T
    x3end = xyz3_in_base[0, 0]
    y3end = xyz3_in_base[0, 1]
    z3end = xyz3_in_base[0, 2]

    #Calculate theta2, 3, 4
    d = z3end - l01
    R = np.sqrt(x3end**2 + y3end**2 + d**2)
    #Theta 2
    alpha = np.arcsin(d / R)
    #Using Cosine Law
    beta = np.arccos((R**2 + l03**2 - l05**2) / (2*l03*R))
    thetas[1] = -alpha-beta
    #Theta3
    gamma = np.arccos((l03**2 + l05**2 - R**2) / (2*l03*l05))
    thetas[2] = np.pi - gamma
    #Theta4
    thetas[3] = -thetas[1] - thetas[2]

    thetas[4]=-PI/2

    print("theta1 to theta6: " + str(thetas) + "\n")

    return thetas

