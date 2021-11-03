import numpy as np
import scipy.interpolate

def compute_smoothed_traj(path, V_des, alpha, dt):
    """
    Fit cubic spline to a path and generate a resulting trajectory for our
    wheeled robot.

    Inputs:
        path (np.array [N,2]): Initial path
        V_des (float): Desired nominal velocity, used as a heuristic to assign nominal
            times to points in the initial path
        alpha (float): Smoothing parameter (see documentation for
            scipy.interpolate.splrep)
        dt (float): Timestep used in final smooth trajectory
    Outputs:
        traj_smoothed (np.array [N,7]): Smoothed trajectory
        t_smoothed (np.array [N]): Associated trajectory times
    Hint: Use splrep and splev from scipy.interpolate
    """
    ########## Code starts here ##########
    # Hint 1 - Determine nominal time for each point in the path using V_des
    # Hint 2 - Use splrep to determine cubic coefficients that best fit given path in x, y
    # Hint 3 - Use splev to determine smoothed paths. The "der" argument may be useful.
    # calculate nominal time for each point

    def get_dt(x_cur, x_prev):
        return np.linalg.norm(x_cur - x_prev) / V_des
    
    def get_splev(t, t_smooth, input):
        print(t.shape)
        print(input.shape)
        x_sprep = scipy.interpolate.splrep(t, input, s=alpha)
        x = scipy.interpolate.splev(t_smooth, x_sprep, der=0)
        xd = scipy.interpolate.splev(t_smooth, x_sprep, der=1)
        xdd = scipy.interpolate.splev(t_smooth, x_sprep, der=2)
        return x, xd, xdd

    path = np.array(path)
    t = np.zeros(len(path))
    for i in range(len(path)):
        t[i] = 0 if i==0 else (t[i-1] + get_dt(path[i, :], path[i-1, :]))
    
    t_smoothed = np.arange(t[0], t[-1], dt)

    x_d, xd_d, xdd_d = get_splev(t, t_smoothed, path[:, 0])
    y_d, yd_d, ydd_d = get_splev(t, t_smoothed, path[:, 1])
    theta_d = np.arctan2(yd_d, xd_d)
    
    ########## Code ends here ##########
    traj_smoothed = np.stack([x_d, y_d, theta_d, xd_d, yd_d, xdd_d, ydd_d]).transpose()
    
    return traj_smoothed, t_smoothed

