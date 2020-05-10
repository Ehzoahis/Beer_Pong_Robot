import numpy as np
import scipy.linalg as la

def string_to_array(string):
    str_list = string.split(sep=" ")
    array = []
    for num in str_list:
        array.append(eval(num))
    return np.array(array)

def omega_v_to_s(B):
    omega = B[:3]
    v = B[3:]
    omega_1 = omega[0, 0]
    omega_2 = omega[1, 0]
    omega_3 = omega[2, 0]

    omega_bracket = np.array([[0, -omega_3, omega_2], [omega_3, 0, - omega_1], [-omega_2, omega_1, 0]])

    t = np.concatenate((omega_bracket, v), axis=1)
    s = np.array([[0, 0, 0, 0]])
    s_bracket = np.concatenate((t, s), axis=0)
    return s_bracket

def adj(T):
    r = T[:3, :3]
    p = T[:3, 3]

    p_1 = p[0]
    p_2 = p[1]
    p_3 = p[2]

    p_bracket = np.array([[0, -p_3, p_2], [p_3, 0, - p_1], [-p_2, p_1, 0]])

    z = np.zeros((3,3))
    pr = np.dot(p_bracket, r)

    ad_t = np.concatenate((r, z), axis=1)
    ad_b = np.concatenate((pr, r), axis=1)
    ad = np.concatenate((ad_t, ad_b), axis=0)
    return ad


def main():
    s = []
    theta_string = input("Enter theta, separate with space: ")
    theta_array = string_to_array(theta_string)

    joint_num = len(theta_array)
    for i in range(joint_num):
        print("Joint", i, ": ")
        joint_type = input("Enter joint type, R for revolution, P for Prismatic: ")
        if joint_type == "R" or joint_type == "r":
            omega_string = input("Enter omega vector, separate with space: ")
            omega_array = string_to_array(omega_string)
            q_string = input("Enter a point on rotation axis, separate with space: ")
            q_array = string_to_array(q_string)
            v = np.cross(-omega_array, q_array)
            omega_array = omega_array.reshape((3, 1))
            v = v.reshape((3, 1))
            s_i = np.concatenate((omega_array, v), axis=0)
        elif joint_type == "P" or joint_type == "p":
            v_string = input("Enter v vector, separate with space: ")
            v_array = string_to_array(v_string)
            v_array = v_array.reshape((3, 1))
            omega = np.zeros(3)
            omega = omega.reshape((3, 1))
            s_i = np.concatenate((omega, v_array), axis=0)
        else:
            print("Invalid input.")
            return
        s.append(s_i)
        print("Si = \n", s_i)

    J = []
    for i in range(joint_num):
        poe = 1
        for j in range(i):
            s_b = omega_v_to_s(s[j])
            s_theta = s_b * theta_array[j]
            poe = np.dot(poe, la.expm(s_theta))
        if i != 0:
            ad = adj(poe)
            J.append(np.dot(ad, s[i]))
        else:
            J.append(s[i])

    J = np.array(J).T
    print("J = \n", J)
    return

main()
