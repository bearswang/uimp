#! /usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from initial import initial_platoon  #
from solve import solve_platoon  # assume solve_platoon function is implemented
import sys

# close figures and clear variables (no clc/clear all needed in Python)
plt.close('all')

delta_t = 0.05  # controller sampling time
T = 100  # total time steps
N_U = 3
max_N_U = 6
max_iteration = 1  # 原始值 = 20
N = 1  # MPC预测步长

# for N_U in range(3, max_N_U+1):
print('Initialization')
z_ref = initial_platoon(T, N_U)
z_n = z_ref.copy()

plt.figure(2)
plt.plot(z_ref[0, :, 0], z_ref[1, :, 0], 'y-', label='Reference trajectory for reference vehicle')
plt.plot(z_ref[0, :, 1], z_ref[1, :, 1], 'y-', label='Real trajectory of ego vehicle 2')
plt.plot(z_ref[0, :, 2], z_ref[1, :, 2], 'y-', label='Real trajectory of ego vehicle 1')
plt.legend()
plt.xlabel('Location in X-coordinate (m)')
plt.ylabel('Location in Y-coordinate (m)')
plt.grid(True)

# initialize storage arrays - corrected dimensions
elapsedtime_tot = np.zeros((max_iteration, T - N))  # length T - N = 99
dist_check_tot = np.zeros((max_iteration, T - N))   # length T - N = 99
record_zn = np.zeros((z_n.shape[0], z_n.shape[1], z_n.shape[2], max_iteration))
record_un = np.zeros((2, T, N_U, max_iteration))  # assume u_n is 2-dimensional control input

for iter in range(max_iteration):
    start = f'start iter: {iter + 1}'
    print(start)

    z_prev = z_n.copy()
    z_n, u_n, elapsedtime_t, dist_check_t = solve_platoon(z_ref, N_U, T)

    elapsedtime_tot[iter, :] = elapsedtime_t[0, :]  # take first row, since elapsedtime_t is (1, T-N)
    dist_check_tot[iter, :] = dist_check_t[:]

    ep = 0
    for t in range(T):
        for user in range(N_U):
            diff = z_n[:, t, user] - z_prev[:, t, user]
            ep += np.dot(diff, diff)

    record_zn[:, :, :, iter] = z_n[:, :, :]
    record_un[:, :, :, iter] = u_n[:, :, :]

    if ep <= 1e-5 or iter == max_iteration - 1:
        print('finished')
        break

print(f'Min distance check: {np.min(dist_check_tot)}')

# plot results for 3 vehicles
plt.figure(2)
plt.clf()  # clear current figure
plt.plot(z_ref[0, :, 0], z_ref[1, :, 0], 'y-', label='Reference trajectory')
plt.plot(z_n[0, :, 0], z_n[1, :, 0], 'r*', label='Real trajectory of vehicle 1')
plt.plot(z_n[0, :, 1], z_n[1, :, 1], 'b*', label='Real trajectory of vehicle 2')
plt.plot(z_n[0, :, 2], z_n[1, :, 2], 'b*', label='Real trajectory of vehicle 3')
plt.legend()
plt.xlabel('Location in X-coordinate (m)')
plt.ylabel('Location in Y-coordinate (m)')
plt.grid(True)

# extract vehicle data
vehicle_ref = z_n[:, :, 0]
vehicle_ego1 = z_n[:, :, 2]
vehicle_ego2 = z_n[:, :, 1]

un_ref = u_n[:, :, 0]
un_ego1 = u_n[:, :, 2]
un_ego2 = u_n[:, :, 1]



def save_vehicle_trajectories_separately(z_ref, z_n):
    
    vehicle_names = ["Lead_Vehicle", "Following_Vehicle_2", "Following_Vehicle_1"]
    
    for vehicle_id in range(N_U):
        # filename = f"./data/refpath_agent_{vehicle_id}.txt"

        # goals
        filename = sys.path[0] + f"/data/refpath_agent_{vehicle_id}.txt"


        with open(filename, 'w') as f:
            f.write(f"{vehicle_names[vehicle_id]} Trajectory Data\n")
            f.write("=" * 50 + "\n")
            f.write("TimeStep, Ref_X, Ref_Y, Actual_X, Actual_Y\n")
            f.write("=" * 50 + "\n")
            
            for t in range(T):
                ref_x = z_ref[0, t, vehicle_id]
                ref_y = z_ref[1, t, vehicle_id]
                actual_x = z_n[0, t, vehicle_id]
                actual_y = z_n[1, t, vehicle_id]
                error_x = actual_x - ref_x
                error_y = actual_y - ref_y
                
                f.write(f"{t:3d} {ref_x:10.6f} {ref_y:10.6f} "
                       f"{actual_x:10.6f} {actual_y:10.6f}\n")


# call function to save data
print("Saving trajectory data to files...")


# save each vehicle's trajectory separately
save_vehicle_trajectories_separately(z_ref, z_n)


print("Trajectory data saved successfully!")
print("Generated files:")
print("  - refpath_agent_0.txt")
print("  - refpath_agent_1.txt") 
print("  - refpath_agent_2.txt")


plt.show()