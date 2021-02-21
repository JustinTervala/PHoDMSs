import numpy as np
from numpy.random import default_rng

def generate_circle(radius, num_points):
    rng = default_rng()
    x_samples = 2 * radius * rng.random(size=num_points) - radius
    y_vals = np.sqrt(radius**2 - x_samples**2)
    return [list(z) for z in zip(x_samples, y_vals)]

def generate_circle_target_dms_file(radius, num_points, num_times, file_name):
    timeseries = []
    for _ in range(num_times):
        timeseries.extend(generate_circle(radius, num_points))
    np.savetxt(file_name, timeseries)
    with open(file_name, "r+") as f:
        content = f.read()
        f.seek(0, 0)
        f.write(str(num_points) + "\n" + content)
    
def generate_initial_position(num_points, file_name, height=250, width=500):
    rng = default_rng()
    x_pos = height * rng.random(size=num_points + 1)
    y_pos = width * rng.random(size=num_points + 1)
    pos = np.vstack((x_pos, y_pos)).T
    np.savetxt(file_name, pos)


if __name__ == "__main__":
    generate_circle_target_dms_file(100, 100, 1000, "circleDMS.txt")
    generate_initial_position(100, "initialDMS.txt")
