from autopilot_msgs.msg import Trajectory, TrajectoryPoint
from astropy.convolution import Gaussian1DKernel, convolve
import copy

class TrajectorySmoother:
    """
    Python version of TrajectorySmoother, inspired from 
    https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/master/src/planning/trajectory_smoother/src/trajectory_smoother.cpp
    """

    def __init__(self, sigma=1, kernal_size=25):
        self._sigma = sigma
        self._kernal_size = kernal_size  # kernal should be Odd number
        # Create kernel
        self._gaussian_kernal = Gaussian1DKernel(stddev=self._sigma, x_size=self._kernal_size)

    def filter(self, traj_in):
        """
        filters the trajectory and 
        returns : Trajectory
        """
        if len(traj_in.points) > 2:
            # zero out velocity at a few points at the end of trajectory so that the post filter velocity
            # gradually ramp down to zero. The last point would have already been zeroed by the
            # estimator.
            zero_run_length = min(len(traj_in.points) // 2, self._kernal_size // 2)
            for i in range(len(traj_in.points) - 1 - zero_run_length, len(traj_in.points)):
                traj_in.points[i].longitudinal_velocity_mps = 0

        # avoid changing the start and end point of trajectory
        # use same padding for points beyond either end of the trajectory
        vel_list = []
        traj_out = copy.deepcopy(traj_in)
        # traj_out = traj_in
        for i in range(0, len(traj_out.points) - 1):
            vel_list.append(traj_out.points[i].longitudinal_velocity_mps)
        # Convolve data
        vel_list = convolve(vel_list, self._gaussian_kernal)
        for i in range(0, len(traj_out.points) - 1):
            traj_out.points[i].longitudinal_velocity_mps = vel_list[i]
        return traj_out


if __name__ == '__main__':
    import matplotlib.pyplot as plt
    ts = TrajectorySmoother(2, 11)
    raw = [2.5] * 100
    raw[0] = 1
    raw[-1] = 0
    traj_in = Trajectory()
    for i in range(len(raw)):
        traj_in.points.append(TrajectoryPoint(longitudinal_velocity_mps=raw[i]))
    for _ in range(1000):
        print(traj_in.points[50].longitudinal_velocity_mps)
        filtered = ts.filter(traj_in)
    filtered_list = []
    for i in range(len(filtered.points)):
        filtered_list.append(filtered.points[i].longitudinal_velocity_mps)

    plt.plot(raw[1:])
    plt.plot(filtered_list[1:])
    print("raw", raw)
    print("filtered_list", filtered_list)

    plt.show()
