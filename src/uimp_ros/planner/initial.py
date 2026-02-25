import numpy as np


def initial_platoon(T, N_U):
    nz = 4
    z_ref = np.zeros((nz, T, N_U))
    v_max = 15  # road speed limit (m/s)
    delta_t = 0.05

    # road geometry
    noLane = 3
    laneWidth = 3.7  # typical US lane width
    road_right = 0
    road_left = road_right + noLane * laneWidth

    x_init = np.array([36, 30, 24, 15, 8, 3])
    y_init = np.array([1, 2, 3, 1, 2, 3])
    # x_init = np.array([36, 30])
    # y_init = np.array([1, 2])

    # initialize each vehicle's initial state
    for user in range(N_U):
        z_ref[0, 0, user] = x_init[user]
        z_ref[1, 0, user] = (y_init[user] - 0.5) * laneWidth
        z_ref[2, 0, user] = 0
        z_ref[3, 0, user] = v_max

    # Phase 1: straight driving
    for i in range(1, int(T / 4)):
        for user in range(N_U):
            z_ref[0, i, user] = z_ref[0, i - 1, user] + v_max * delta_t
            z_ref[1, i, user] = z_ref[1, i - 1, user]
            z_ref[2, i, user] = 0
            z_ref[3, i, user] = np.sqrt((z_ref[0, i, user] - z_ref[0, i - 1, user]) ** 2 +
                                        (z_ref[1, i, user] - z_ref[1, i - 1, user]) ** 2)

    # Phase 2: lane change
    for i in range(int(T / 4), int(3 * T / 4)):
        for user in range(N_U):
            z_ref[0, i, user] = z_ref[0, i - 1, user] + v_max * delta_t
            z_ref[2, i, user] = 0

            if y_init[user] == 3:
                z_ref[1, i, user] = z_ref[1, i - 1, user] - (y_init[user] - 1) * laneWidth / (3 * T / 4 - T / 4)
            elif y_init[user] == 2 and i < int(T / 2):
                z_ref[1, i, user] = z_ref[1, i - 1, user] - (y_init[user] - 1) * laneWidth / (T / 2 - T / 4)
            else:
                z_ref[1, i, user] = z_ref[1, i - 1, user]

            z_ref[3, i, user] = np.sqrt((z_ref[0, i, user] - z_ref[0, i - 1, user]) ** 2 +
                                        (z_ref[1, i, user] - z_ref[1, i - 1, user]) ** 2)

    # Phase 3: straight driving
    for i in range(int(3 * T / 4), T):
        for user in range(N_U):
            z_ref[0, i, user] = z_ref[0, i - 1, user] + v_max * delta_t
            z_ref[1, i, user] = z_ref[1, i - 1, user]
            z_ref[2, i, user] = 0
            z_ref[3, i, user] = np.sqrt((z_ref[0, i, user] - z_ref[0, i - 1, user]) ** 2 +
                                        (z_ref[1, i, user] - z_ref[1, i - 1, user]) ** 2)

    # # implement communication delay for vehicle 3 (example)
    # z_ref[:, :, 2] = np.zeros((nz, T))
    # delay = 7  # represents 70ms communication delay
    #
    # z_ref[0, 0, 2] = x_init[2]
    # z_ref[1, 0, 2] = (y_init[2] - 0.5) * laneWidth
    # z_ref[3, 0, 2] = v_max
    #
    # for i in range(1, int(T/4 + delay)):
    #     z_ref[0, i, 2] = z_ref[0, i-1, 2] + (v_max-3) * delta_t
    #     z_ref[1, i, 2] = z_ref[1, i-1, 2]
    #     z_ref[3, i, 2] = np.sqrt((z_ref[0, i, 2] - z_ref[0, i-1, 2])**2 +
    #                            (z_ref[1, i, 2] - z_ref[1, i-1, 2])**2)
    #
    # for i in range(int(T/4 + delay), int(3*T/4 + delay)):
    #     z_ref[0, i, 2] = z_ref[0, i-1, 2] + (v_max+5) * delta_t
    #     z_ref[1, i, 2] = z_ref[1, i-1, 2] - (y_init[2] - 1) * laneWidth / (3*T/4 - T/4)
    #     z_ref[3, i, 2] = np.sqrt((z_ref[0, i, 2] - z_ref[0, i-1, 2])**2 +
    #                            (z_ref[1, i, 2] - z_ref[1, i-1, 2])**2)
    #
    # for i in range(int(3*T/4 + delay), T):
    #     z_ref[0, i, 2] = z_ref[0, i-1, 2] + v_max * delta_t
    #     z_ref[1, i, 2] = z_ref[1, i-1, 2]
    #     z_ref[3, i, 2] = np.sqrt((z_ref[0, i, 2] - z_ref[0, i-1, 2])**2 +
    #                            (z_ref[1, i, 2] - z_ref[1, i-1, 2])**2)

    return z_ref