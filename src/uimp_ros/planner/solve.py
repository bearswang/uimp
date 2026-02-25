import numpy as np
import matplotlib.pyplot as plt
from rotation import rotation_translation
from yalmip import mpc_yalmip


def solve_platoon(z_ref, N_U, T):
    """
    Solve the platoon MPC trajectory planning problem.

    Parameters:
    z_ref: reference trajectories
    N_U: number of vehicles
    T: total number of time steps

    Returns:
    z_traj: state trajectories
    u_traj: control input trajectories
    elapsedtime_t: solve time per step
    dist_check_t: distance checks per step
    """
    nA = 4
    nb = 4
    nz = 4  # 状态数量：位置(x,y)、航向psi、速度v。z = [x,y,psi,v]
    nu = 2  # 输入数量：加速度和转向控制输入。u = [a,delta_f]
    N = 1

    l_f = 4.7 / 2  # front half-length of vehicle
    l_r = 4.7 / 2  # rear half-length of vehicle
    h = l_f + l_r  # vehicle total length
    w = 1.85  # vehicle width

    z_traj = np.zeros((nz, T, N_U))
    u_traj = np.zeros((nu, T, N_U))
    z_prev = z_ref[:, 0:1, :].copy()
    u_prev = np.zeros((nu, 1, N_U))
    z_traj[:, 0:1, :] = z_prev
    A1 = np.zeros((nA, 2, 1, N_U))
    b1 = np.zeros((nb, 1, N_U))
    elapsedtime_t = np.zeros((1, T - N))

    # commented-out section
    # delay = 7
    # ratio_of_acc = 15 / 2  # fast merging parameter

    # # Effect of V2V communication delay on vehicle 3 (example)
    # for i in range(T):
    #     # commented-out logic
    #     # if (i >= T//2+delay) and (i <= T//2+delay+N):
    #     #     z_ref[:, i, 2] = z_ref[:, i-N, 2]

    #     if (i >= T // 4 + delay) and (i <= T // 4 + delay + 17 * N):
    #         z_ref[:, i, 2] = z_ref[:, i - N, 2]
    #         z_ref[3, i, 2] = z_ref[3, i, 2] + ratio_of_acc

    # ratio_of_deacc = 15/2  # parameter used for MPC

    # lidar-induced deceleration (commented out)
    # for i in range(T):
    #     if (i >= T//4-N) and (i <= T//4+N):
    #         z_ref[0:2, i, 2] = z_ref[0:2, i, 2] - z_ref[0:2, i, 2]/2  # 2 or 7.5
    #         z_ref[3, i, 2] = z_ref[3, i, 2] - ratio_of_deacc

    # initialize distance check array
    dist_check_t = np.zeros(T - N)

    # main loop
    for i in range(T - N):
        start = f'start algorithm iter: {i + 1}'
        print(start)

        for user in range(N_U):
            A1[:, :, 0, user], b1[:, 0, user] = rotation_translation(
                np.array([z_prev[0, 0, user], z_prev[1, 0, user]]),
                z_prev[2, 0, user], h, w)

        z_k, u_k, elapsedtime, distance = mpc_yalmip(
            z_prev, u_prev, z_ref[:, i:i + N + 1, :], N_U, N, A1, b1)

        # check whether MPC solved successfully
        if z_k is None or u_k is None:
            print(f"MPC solve failed, stopping at iteration {i+1}")
            print(f"Continuing with reference trajectory...")
            # use reference trajectory as a fallback
            z_k = z_ref[:, i:i + N + 1, :].copy()
            u_k = np.zeros((nu, N, N_U))
            distance = np.inf
            
        # store trajectories
        z_traj[:, i + 1:i + N + 1, :] = z_k[:, 1:N + 1, :]
        print(z_k[0:2, 1, :])
        u_traj[:, i:i + N, :] = u_k[:, 0:N, :]
        z_prev = z_k[:, 1:2, :]  # keep 3-D structure (nz, 1, N_U)
        u_prev = u_k[:, 0:1, :]  # keep 3-D structure (nu, 1, N_U)
        elapsedtime_t[0, i] = elapsedtime
        print(f'iter: {i}')
        print(f'distance: {distance}')
        dist_check_t[i] = distance

    # plot trajectories of 3 vehicles
    plt.figure(1)
    plt.clf()  # clear previous figure
    plt.plot(z_ref[0, :, 0], z_ref[1, :, 0], 'y-', label='Reference')
    plt.plot(z_k[0, :, 0], z_k[1, :, 0], 'b*', label='Vehicle 1')
    plt.plot(z_k[0, :, 1], z_k[1, :, 1], 'r*', label='Vehicle 2')
    plt.plot(z_k[0, :, 2], z_k[1, :, 2], 'm*', label='Vehicle 3')
    plt.plot([30, 110], [3.7, 3.7], 'y--', label='Lane boundary')
    plt.plot([30, 110], [7.4, 7.4], 'y--')
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.legend()
    plt.grid(True)
    plt.pause(0.01)  # real-time display

    return z_traj, u_traj, elapsedtime_t, dist_check_t