
import numpy as np
import casadi as ca
import time
from rotation import rotation_translation


def mpc_yalmip(z_prev, u_prev, z_ref, N_U, N, A1, b1):
    """
    MPC controller implemented using CasADi
    """
    # model and MPC data and variables
    nz = 4  # 状态数量：位置(x,y)、航向psi、速度v。z = [x,y,psi,v]
    nu = 2  # number of inputs: acceleration and steering input. u = [a, delta_f]

    g = 9.81  # gravity acceleration
    delta_t = 0.05  # controller sampling time
    d_min = 0.3  # minimum distance between vehicles (m), design parameter
    v_max = 15  # road speed limit (m/s)
    v_threshold = 4

    steer_limit = 0.3  # steering angle limit (radians)
    accel_limit = 4  # acceleration limit (m/s^2)
    change_steer_limit = 0.2  # steering rate limit
    change_accel_limit = 0.3  # acceleration rate limit

    # road geometry
    noLane = 3
    laneWidth = 3.7  # 美国道路宽度标准
    road_right = 0
    road_left = road_right + noLane * laneWidth

    # polytope dimensions
    nlambda = 4
    ns = 2
    nA = 4
    nb = 4

    l_f = 4.7 / 2  # front half-length of the vehicle
    l_r = 4.7 / 2  # rear half-length of the vehicle
    h = l_f + l_r  # vehicle total length
    w = 1.85  # vehicle width

    Q = 0.1 * np.diag([1, 100, 1, 0.1])
    R = 0.1 * np.diag([1, 1])

    # compute total number of optimization variables
    n_z = nz * (N + 1) * N_U
    n_u = nu * N * N_U
    n_belta = N * N_U
    n_A = nA * 2 * (N + 1) * N_U
    n_b = nb * (N + 1) * N_U
    n_lambda = nlambda * (N + 1) * N_U * N_U
    n_mu = nlambda * (N + 1) * N_U * N_U
    n_s = ns * (N + 1) * N_U * N_U

    total_vars = n_z + n_u + n_belta + n_A + n_b + n_lambda + n_mu + n_s

    # define 1-D optimization variable vector
    vars_vec = ca.MX.sym('vars', total_vars)

    # define index mapping helper functions
    def z_idx(i, k, user):
        return i * (N + 1) * N_U + k * N_U + user

    def u_idx(i, k, user):
        return n_z + i * N * N_U + k * N_U + user

    def belta_idx(k, user):
        return n_z + n_u + k * N_U + user

    def A_idx(i, j, k, user):
        return n_z + n_u + n_belta + i * 2 * (N + 1) * N_U + j * (N + 1) * N_U + k * N_U + user

    def b_idx(i, k, user):
        return n_z + n_u + n_belta + n_A + i * (N + 1) * N_U + k * N_U + user

    def lambda_idx(i, k, user1, user2):
        return n_z + n_u + n_belta + n_A + n_b + i * (N + 1) * N_U * N_U + k * N_U * N_U + user1 * N_U + user2

    def mu_idx(i, k, user1, user2):
        return n_z + n_u + n_belta + n_A + n_b + n_lambda + i * (N + 1) * N_U * N_U + k * N_U * N_U + user1 * N_U + user2

    def s_idx(i, k, user1, user2):
        return n_z + n_u + n_belta + n_A + n_b + n_lambda + n_mu + i * (N + 1) * N_U * N_U + k * N_U * N_U + user1 * N_U + user2

    # constraint lists
    constraints = []
    lbg = []  # lower bounds for constraints
    ubg = []  # upper bounds for constraints

    # initial condition constraints
    for user in range(N_U):
        # A初始条件
        for i in range(nA):
            for j in range(2):
                constraints.append(vars_vec[A_idx(i, j, 0, user)] - A1[i, j, 0, user])
                lbg.append(0)
                ubg.append(0)
        
        # b initial condition
        for i in range(nb):
            constraints.append(vars_vec[b_idx(i, 0, user)] - b1[i, 0, user])
            lbg.append(0)
            ubg.append(0)
        
        # z initial condition
        for i in range(nz):
            constraints.append(vars_vec[z_idx(i, 0, user)] - z_prev[i, 0, user])
            lbg.append(0)
            ubg.append(0)

    # dynamics constraints
    for k in range(N):
        for user in range(N_U):
            # belta computation constraint
            belta_expected = (l_r / (l_f + l_r)) * ca.atan(vars_vec[u_idx(1, k, user)])
            constraints.append(vars_vec[belta_idx(k, user)] - belta_expected)
            lbg.append(0)
            ubg.append(0)

            # state update equations
            # x position
            constraints.append(vars_vec[z_idx(0, k + 1, user)] - 
                             (vars_vec[z_idx(0, k, user)] + delta_t * vars_vec[z_idx(3, k, user)]))
            lbg.append(0)
            ubg.append(0)

            # y position
            constraints.append(vars_vec[z_idx(1, k + 1, user)] - 
                             (vars_vec[z_idx(1, k, user)] + delta_t * vars_vec[z_idx(3, k, user)] * 
                              (vars_vec[z_idx(2, k, user)] + vars_vec[belta_idx(k, user)])))
            lbg.append(0)
            ubg.append(0)

            # heading angle
            constraints.append(vars_vec[z_idx(2, k + 1, user)] - 
                             (vars_vec[z_idx(2, k, user)] + delta_t * (vars_vec[z_idx(3, k, user)] / l_r) * 
                              vars_vec[belta_idx(k, user)]))
            lbg.append(0)
            ubg.append(0)

            # speed
            constraints.append(vars_vec[z_idx(3, k + 1, user)] - 
                             (vars_vec[z_idx(3, k, user)] + delta_t * vars_vec[u_idx(0, k, user)]))
            lbg.append(0)
            ubg.append(0)

    # road boundary constraints
    for k in range(N + 1):
        for user in range(N_U):
            # left boundary
            constraints.append(vars_vec[z_idx(1, k, user)] + w / 2)
            lbg.append(-ca.inf)
            ubg.append(road_left)
            
            # right boundary
            constraints.append(-vars_vec[z_idx(1, k, user)] + w / 2)
            lbg.append(-ca.inf)
            ubg.append(-road_right)
            
            # speed constraint
            constraints.append(vars_vec[z_idx(3, k, user)])
            lbg.append(0.0)
            ubg.append(v_max + v_threshold)

    # control input constraints
    for k in range(N):
        for user in range(N_U):
            # acceleration constraint
            constraints.append(vars_vec[u_idx(0, k, user)])
            lbg.append(-accel_limit)
            ubg.append(accel_limit)
            
            # steering angle constraint
            constraints.append(vars_vec[u_idx(1, k, user)])
            lbg.append(-steer_limit)
            ubg.append(steer_limit)

    # control input rate-of-change constraints
    for k in range(1, N):
        for user in range(N_U):
            # acceleration change constraint
            constraints.append(vars_vec[u_idx(0, k, user)] - vars_vec[u_idx(0, k - 1, user)])
            lbg.append(-change_accel_limit)
            ubg.append(change_accel_limit)
            
            # steering change constraint
            constraints.append(vars_vec[u_idx(1, k, user)] - vars_vec[u_idx(1, k - 1, user)])
            lbg.append(-change_steer_limit)
            ubg.append(change_steer_limit)

    # initial control input change constraints
    for user in range(N_U):
        # acceleration change constraint
        constraints.append(vars_vec[u_idx(0, 0, user)] - u_prev[0, 0, user])
        lbg.append(-change_accel_limit)
        ubg.append(change_accel_limit)
        
        # steering change constraint
        constraints.append(vars_vec[u_idx(1, 0, user)] - u_prev[1, 0, user])
        lbg.append(-change_steer_limit)
        ubg.append(change_steer_limit)

    # vehicle geometry constraints
    for k in range(N + 1):
        for user in range(N_U):
            # compute rotation matrix
            theta = vars_vec[z_idx(2, k, user)]
            cos_theta = ca.cos(theta)
            sin_theta = ca.sin(theta)
            
            # vehicle center position
            x_pos = vars_vec[z_idx(0, k, user)]
            y_pos = vars_vec[z_idx(1, k, user)]
            
            # compute A matrix elements (4x2)
            A_calc = ca.vertcat(
                ca.horzcat(cos_theta, sin_theta),
                ca.horzcat(-sin_theta, cos_theta),
                ca.horzcat(-cos_theta, -sin_theta),
                ca.horzcat(sin_theta, -cos_theta)
            )
            
            # compute b vector
            b_calc = ca.vertcat(
                h/2 + cos_theta * x_pos + sin_theta * y_pos,
                w/2 - sin_theta * x_pos + cos_theta * y_pos,
                h/2 - cos_theta * x_pos - sin_theta * y_pos,
                w/2 + sin_theta * x_pos - cos_theta * y_pos
            )
            
            # add geometry constraints
            for i in range(nA):
                for j in range(2):
                    constraints.append(vars_vec[A_idx(i, j, k, user)] - A_calc[i, j])
                    lbg.append(0)
                    ubg.append(0)
            
            for i in range(nb):
                constraints.append(vars_vec[b_idx(i, k, user)] - b_calc[i])
                lbg.append(0)
                ubg.append(0)

    # collision avoidance constraints
    for k in range(N + 1):
        for user1 in range(N_U):
            if user1 != 4:  # user1 != 5 in MATLAB (0-indexed)
                for user2 in range(N_U):
                    if user1 != user2:
                        # separating hyperplane constraints
                        # first constraint: b1^T * lambda + b2^T * mu <= -d_min
                        sum_constraint = 0
                        for i in range(nb):
                            sum_constraint += vars_vec[b_idx(i, k, user1)] * vars_vec[lambda_idx(i, k, user1, user2)]
                            sum_constraint += vars_vec[b_idx(i, k, user2)] * vars_vec[mu_idx(i, k, user1, user2)]
                        
                        constraints.append(sum_constraint)
                        lbg.append(-ca.inf)
                        ubg.append(-d_min)
                        
                        # second constraint: A1^T * lambda + s = 0
                        for dim in range(ns):
                            sum_constraint = 0
                            for i in range(nA):
                                sum_constraint += vars_vec[A_idx(i, dim, k, user1)] * vars_vec[lambda_idx(i, k, user1, user2)]
                            sum_constraint += vars_vec[s_idx(dim, k, user1, user2)]
                            
                            constraints.append(sum_constraint)
                            lbg.append(0)
                            ubg.append(0)
                        
                        # third constraint: A2^T * mu - s = 0
                        for dim in range(ns):
                            sum_constraint = 0
                            for i in range(nA):
                                sum_constraint += vars_vec[A_idx(i, dim, k, user2)] * vars_vec[mu_idx(i, k, user1, user2)]
                            sum_constraint -= vars_vec[s_idx(dim, k, user1, user2)]
                            
                            constraints.append(sum_constraint)
                            lbg.append(0)
                            ubg.append(0)
                        
                        # lambda <= 0 constraints
                        for i in range(nlambda):
                            constraints.append(vars_vec[lambda_idx(i, k, user1, user2)])
                            lbg.append(-ca.inf)
                            ubg.append(0)
                        
                        # mu <= 0 constraints
                        for i in range(nlambda):
                            constraints.append(vars_vec[mu_idx(i, k, user1, user2)])
                            lbg.append(-ca.inf)
                            ubg.append(0)
                        
                        # ||s||^2 <= 1 constraint
                        s_norm_sq = (vars_vec[s_idx(0, k, user1, user2)]**2 + 
                                    vars_vec[s_idx(1, k, user1, user2)]**2)
                        constraints.append(s_norm_sq)
                        lbg.append(-ca.inf)
                        ubg.append(1)

    # objective function
    obj = 0

    # tracking error and control cost
    for k in range(N):
        for user in range(N_U):
            # state error
            error = ca.vertcat(
                vars_vec[z_idx(0, k, user)] - z_ref[0, k, user],
                vars_vec[z_idx(1, k, user)] - z_ref[1, k, user],
                vars_vec[z_idx(2, k, user)] - z_ref[2, k, user],
                vars_vec[z_idx(3, k, user)] - z_ref[3, k, user]
            )
            
            # control inputs
            u_current = ca.vertcat(
                vars_vec[u_idx(0, k, user)],
                vars_vec[u_idx(1, k, user)]
            )
            
            obj += ca.mtimes([error.T, Q, error]) + ca.mtimes([u_current.T, R, u_current])

    # terminal cost
    for user in range(N_U):
        error = ca.vertcat(
            vars_vec[z_idx(0, N, user)] - z_ref[0, N, user],
            vars_vec[z_idx(1, N, user)] - z_ref[1, N, user],
            vars_vec[z_idx(2, N, user)] - z_ref[2, N, user],
            vars_vec[z_idx(3, N, user)] - z_ref[3, N, user]
        )
        obj += ca.mtimes([error.T, Q, error])

    # additional terminal cost
    for k in range(1, N + 1):
        for user in range(1, N_U):
            error = ca.vertcat(
                vars_vec[z_idx(0, N, user)] - z_ref[0, N, user],
                vars_vec[z_idx(1, N, user)] - z_ref[1, N, user],
                vars_vec[z_idx(2, N, user)] - z_ref[2, N, user],
                vars_vec[z_idx(3, N, user)] - z_ref[3, N, user]
            )
            obj += ca.mtimes([error.T, Q, error])

    # set variable bounds
    lbx = [-ca.inf] * total_vars
    ubx = [ca.inf] * total_vars
    
    # z_k bounds
    for i in range(nz):
        for k in range(N + 1):
            for user in range(N_U):
                idx = z_idx(i, k, user)
                if i == 3:  # 速度
                    lbx[idx] = 0.0
                    ubx[idx] = v_max + v_threshold
    
    # u_k bounds
    for i in range(nu):
        for k in range(N):
            for user in range(N_U):
                idx = u_idx(i, k, user)
                if i == 0:  # 加速度
                    lbx[idx] = -accel_limit
                    ubx[idx] = accel_limit
                else:  # 转向角
                    lbx[idx] = -steer_limit
                    ubx[idx] = steer_limit

    # create NLP problem
    if len(constraints) > 0:
        constraints_vec = ca.vertcat(*constraints)
    else:
        constraints_vec = ca.MX.zeros(0, 1)
    
    lbg = np.array(lbg)
    ubg = np.array(ubg)
    lbx = np.array(lbx)
    ubx = np.array(ubx)

    nlp = {
        'x': vars_vec,
        'f': obj,
        'g': constraints_vec
    }

    # solver options
    opts = {
        'ipopt': {
            'print_level': 1,
            'max_iter': 1000,
            'tol': 1e-6,
            'acceptable_tol': 1e-4,
            'mu_strategy': 'adaptive',
            'hessian_approximation': 'limited-memory'
        },
        'print_time': False
    }

    # create solver
    solver = ca.nlpsol('solver', 'ipopt', nlp, opts)

    # initial guess
    x0 = np.zeros(total_vars)
    
    # set initial state values
    for i in range(nz):
        for k in range(N + 1):
            for user in range(N_U):
                idx = z_idx(i, k, user)
                if k == 0:
                    x0[idx] = z_prev[i, 0, user]
                else:
                    x0[idx] = z_ref[i, k, user]  # 使用参考轨迹作为初始猜测

    # set initial control inputs
    for i in range(nu):
        for k in range(N):
            for user in range(N_U):
                idx = u_idx(i, k, user)
                if k == 0:
                    x0[idx] = u_prev[i, 0, user]
                else:
                    x0[idx] = 0.0

    # initialize A matrix
    for i in range(nA):
        for j in range(2):
            for k in range(N + 1):
                for user in range(N_U):
                    idx = A_idx(i, j, k, user)
                    x0[idx] = A1[i, j, 0, user]

    # initialize b vector
    for i in range(nb):
        for k in range(N + 1):
            for user in range(N_U):
                idx = b_idx(i, k, user)
                x0[idx] = b1[i, 0, user]

    # start timing
    start_time = time.time()

    try:
    # solve the optimization problem
        sol = solver(x0=x0, lbx=lbx, ubx=ubx, lbg=lbg, ubg=ubg)
        elapsedtime = time.time() - start_time

    # check solver status
        if solver.stats()['return_status'] in ['Solve_Succeeded', 'Solved_To_Acceptable_Level']:
            # extract solution
            x_opt = sol['x'].full().flatten()
            
            # parse states and control inputs
            z_k_value = np.zeros((nz, N + 1, N_U))
            u_k_value = np.zeros((nu, N, N_U))
            
            # extract states
            for i in range(nz):
                for k in range(N + 1):
                    for user in range(N_U):
                        z_k_value[i, k, user] = x_opt[z_idx(i, k, user)]
            
            # extract control inputs
            for i in range(nu):
                for k in range(N):
                    for user in range(N_U):
                        u_k_value[i, k, user] = x_opt[u_idx(i, k, user)]

            # compute distance
            distance = np.sqrt((z_k_value[0, 0, 1] - z_k_value[0, 0, 2]) ** 2 +
                               (z_k_value[1, 0, 1] - z_k_value[1, 0, 2]) ** 2) - 4.47

            return z_k_value, u_k_value, elapsedtime, distance
        else:
            print(f"IPOPT solve failed, status: {solver.stats()['return_status']}")
            return None, None, elapsedtime, np.inf

    except Exception as e:
        print(f"Error occurred during solving: {e}")
        return None, None, time.time() - start_time, np.inf