import math
import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp
import matplotlib.pyplot as plt

def linear_interpolation(intial_p, target_p, velocity, base_step_size=2):
    interpolated_points = []
    interpolated_points.append(intial_p)

    # Convert orientation from degrees to radians
    P1_orientation_rad = np.radians(intial_p[3:])
    P2_orientation_rad = np.radians(target_p[3:])

    # Convert Euler angles to quaternions
    P1_quat = R.from_euler('xyz', P1_orientation_rad).as_quat()
    P2_quat = R.from_euler('xyz', P2_orientation_rad).as_quat()

    # Calculate the total distance
    position_distance = np.linalg.norm(np.array(target_p[:3]) - np.array(intial_p[:3]))
    orientation_distance = R.from_quat(P1_quat).inv() * R.from_quat(P2_quat)
    orientation_distance = orientation_distance.magnitude()
    total_distance = position_distance + orientation_distance
    
    # Calculate the total time
    total_time = total_distance / velocity

    # Calculate the number of steps based on the base step size
    steps = int(total_distance / base_step_size)

    if steps <= 0:
        steps = 1

    # Calculate the time per step
    time_per_step = total_time / steps
    print(f'total_time: {time_per_step}')
    
    
    key_times = [0, 1]
    key_rots = R.from_quat([P1_quat, P2_quat])
    slerp = Slerp(key_times, key_rots)
    
    for i in range(steps + 1):

        # how to solve if steps = 0:
        if steps == 0:
            t = 1
        else:
            t = i / (steps)
        
        
        # Interpolate the position
        interpolated_pos = [intial_p[j] + t * (target_p[j] - intial_p[j]) for j in range(3)]
        interpolated_quat = slerp(t).as_quat()

        # Convert the quaternion to Euler angles
        orientation_rad = R.from_quat(interpolated_quat).as_euler('xyz')
        orientation_deg = np.degrees(orientation_rad)
        # Append the interpolated point
        interpolated_points.append([interpolated_pos[0], interpolated_pos[1], interpolated_pos[2], orientation_deg[0], orientation_deg[1], orientation_deg[2]])

    return interpolated_points, steps, time_per_step

def joint_interpolation(initial_joints, target_joints, speed):
    # Calculate the number of steps based on the speed
    max_joint_diff = max([abs(target_joints[i] - initial_joints[i]) for i in range(len(initial_joints))])
    steps = int(max_joint_diff / speed * 100)  # Adjust the factor as needed for your speed unit
    time_per_step = 1.0 / steps  # Assuming speed is in units per second

    # Interpolate joint positions
    interpolated_joints = []
    for step in range(steps + 1):
        interpolated_step = [
            initial_joints[i] + (target_joints[i] - initial_joints[i]) * step / steps
            for i in range(len(initial_joints))
        ]
        interpolated_joints.append(interpolated_step)
    return interpolated_joints, steps, time_per_step    

def plot_interpolated_points(interpolated_points):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Extract position and orientation data
    x = [point[0] for point in interpolated_points]
    y = [point[1] for point in interpolated_points]
    z = [point[2] for point in interpolated_points]
    orientations = [point[3:] for point in interpolated_points]

    # Plot the interpolated points
    ax.plot(x, y, z, label='Interpolated Path')

    # Plot the orientation frames
    for i in range(len(interpolated_points)):
        pos = np.array([x[i], y[i], z[i]])
        orientation_rad = np.radians(orientations[i])
        rot = R.from_euler('xyz', orientation_rad)
        frame = rot.as_matrix()

        # Plot the orientation frame
        ax.quiver(pos[0], pos[1], pos[2], frame[0, 0], frame[0, 1], frame[0, 2], color='r', length=10)
        ax.quiver(pos[0], pos[1], pos[2], frame[1, 0], frame[1, 1], frame[1, 2], color='g', length=10)
        ax.quiver(pos[0], pos[1], pos[2], frame[2, 0], frame[2, 1], frame[2, 2], color='b', length=10)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    plt.show()

def test_linear_interpolation():
    P1 = [500, -200, 400, 180, 0, 0]
    P2 = [600, -200, 400, 90, 0, 0]
    velocity = 10
    interpolated_points, steps, time_per_step = linear_interpolation(P1, P2, velocity)
    
    print(f'velocity: {velocity}')  
    print(f'steps: {steps}')
    print(f'time_per_step: {time_per_step}')

    plot_interpolated_points(interpolated_points)

if __name__ == '__main__':
    test_linear_interpolation()

