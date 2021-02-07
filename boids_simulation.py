# This code is adapted for our purposes from original code online:
# -----------------------------------------------------------------------------
# From Pytnon to Numpy
# Copyright (2017) Nicolas P. Rougier - BSD license
# More information at https://github.com/rougier/numpy-book
# -----------------------------------------------------------------------------
#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.path import Path
from matplotlib.animation import FuncAnimation
from matplotlib.collections import PathCollection
import sys
import os

class MarkerCollection:
    """
    Marker collection
    """

    def __init__(self, n=100):
        v = np.array([(-0.25, -0.25), (+0.0, +0.5), (+0.25, -0.25), (0, 0)])
        c = np.array([Path.MOVETO, Path.LINETO, Path.LINETO, Path.CLOSEPOLY])
        self._base_vertices = np.tile(v.reshape(-1), n).reshape(n, len(v), 2)
        self._vertices = np.tile(v.reshape(-1), n).reshape(n, len(v), 2)
        self._codes = np.tile(c.reshape(-1), n)

        self._scale = np.ones(n)
        self._translate = np.zeros((n, 2))
        self._rotate = np.zeros(n)

        self._path = Path(vertices=self._vertices.reshape(n*len(v), 2),
                          codes=self._codes)
        self._collection = PathCollection([self._path], linewidth=0.5,
                                          facecolor="k", edgecolor="w")

    def update(self):
        n = len(self._base_vertices)
        self._vertices[...] = self._base_vertices * self._scale
        cos_rotate, sin_rotate = np.cos(self._rotate), np.sin(self._rotate)
        R = np.empty((n, 2, 2))
        R[:, 0, 0] = cos_rotate
        R[:, 1, 0] = sin_rotate
        R[:, 0, 1] = -sin_rotate
        R[:, 1, 1] = cos_rotate
        self._vertices[...] = np.einsum('ijk,ilk->ijl', self._vertices, R)
        self._vertices += self._translate.reshape(n, 1, 2)


class Flock:
    def __init__(self, count=500, width=500, height=250):
        self.width = width
        self.height = height
        self.min_velocity = 0.5
        self.max_velocity = 2.0
        self.max_acceleration = 0.03
        self.velocity = np.zeros((count, 2), dtype=np.float32)
        self.position = np.zeros((count, 2), dtype=np.float32)

        angle = np.random.uniform(0, 2*np.pi, count)
        self.velocity[:, 0] = np.cos(angle)
        self.velocity[:, 1] = np.sin(angle)
        angle = np.random.uniform(0, 2*np.pi, count)
        radius = min(width, height)/2*np.random.uniform(0, 1, count)
        self.position[:, 0] = width/2 + np.cos(angle)*radius
        self.position[:, 1] = height/2 + np.sin(angle)*radius


    def set_positions(self, positions):
        self.position = positions


    def run(self):

        global distance, counter, sep, ali, coh

        position = self.position
        velocity = self.velocity
        min_velocity = self.min_velocity
        max_velocity = self.max_velocity
        max_acceleration = self.max_acceleration
        n = len(position)

        dx = np.absolute(np.subtract.outer(position[:, 0], position[:, 0]))
        dx = np.minimum(dx, self.width-dx)
        dy = np.absolute(np.subtract.outer(position[:, 1], position[:, 1]))
        dy = np.minimum(dy, self.height-dy)
        distance = np.hypot(dx, dy)

        # Compute common distance masks
        mask_0 = (distance > 0)
        mask_1 = (distance < seprad)
        mask_2 = (distance < alirad)
        mask_3 = (distance < cohrad)
        mask_1 *= mask_0
        mask_2 *= mask_0
        mask_3 *= mask_0
        mask_1_count = np.maximum(mask_1.sum(axis=1), 1)
        mask_2_count = np.maximum(mask_2.sum(axis=1), 1)
        mask_3_count = np.maximum(mask_3.sum(axis=1), 1)

        # Separation
        mask, count = mask_1, mask_1_count
        target = np.dstack((dx, dy))
        target = np.divide(target, distance.reshape(n, n, 1)**2, out=target,
                           where=distance.reshape(n, n, 1) != 0)
        steer = (target*mask.reshape(n, n, 1)).sum(axis=1)/count.reshape(n, 1)
        norm = np.sqrt((steer*steer).sum(axis=1)).reshape(n, 1)
        steer = max_velocity*np.divide(steer, norm, out=steer,
                                       where=norm != 0)
        steer -= velocity

        # Limit acceleration
        norm = np.sqrt((steer*steer).sum(axis=1)).reshape(n, 1)
        steer = np.multiply(steer, max_acceleration/norm, out=steer,
                            where=norm > max_acceleration)

        separation = steer

        # Alignment
        # ---------------------------------------------------------------------
        # Compute target
        mask, count = mask_2, mask_2_count
        target = np.dot(mask, velocity)/count.reshape(n, 1)

        # Compute steering
        norm = np.sqrt((target*target).sum(axis=1)).reshape(n, 1)
        target = max_velocity * np.divide(target, norm, out=target,
                                          where=norm != 0)
        steer = target - velocity

        # Limit acceleration
        norm = np.sqrt((steer*steer).sum(axis=1)).reshape(n, 1)
        steer = np.multiply(steer, max_acceleration/norm, out=steer,
                            where=norm > max_acceleration)
        alignment = steer

        # Cohesion
        # ---------------------------------------------------------------------
        # Compute target
        mask, count = mask_3, mask_3_count
        target = np.dot(mask, position)/count.reshape(n, 1)

        # Compute steering
        desired = target - position
        norm = np.sqrt((desired*desired).sum(axis=1)).reshape(n, 1)
        desired *= max_velocity / norm
        steer = desired - velocity

        # Limit acceleration
        norm = np.sqrt((steer*steer).sum(axis=1)).reshape(n, 1)
        steer = np.multiply(steer, max_acceleration/norm, out=steer,
                            where=norm > max_acceleration)
        cohesion = steer

        # ---------------------------------------------------------------------
        acceleration = sep*separation + ali*alignment + coh*cohesion
        velocity += acceleration

        norm = np.sqrt((velocity*velocity).sum(axis=1)).reshape(n, 1)
        velocity = np.multiply(velocity, max_velocity/norm, out=velocity,
                               where=norm > max_velocity)
        velocity = np.multiply(velocity, min_velocity/norm, out=velocity,
                               where=norm < min_velocity)
        position += velocity

        # Wraparound
        position += (self.width, self.height)
        position %= (self.width, self.height)


def update(*args):
    global flock, collection, trace, filename, distance, counter

    # Flock updating
    flock.run()
    collection._scale = 10
    collection._translate = flock.position
    collection._rotate = -np.pi/2 + np.arctan2(flock.velocity[:, 1],
                                               flock.velocity[:, 0])
    collection.update()

    if ctrl_callback:
        ctrl_callback(flock.position)

    #collect positions
    f = open(filename, "a")
    np.savetxt(f, flock.position, fmt='%.8f', newline='\n')
    f.close()

    # Trace updating
    if trace is not None:
        P = flock.position.astype(int)
        trace[height-1-P[:, 1], P[:, 0]] = .75
        trace *= .99
        im.set_array(trace)


def run_sim(
    num_boids,
    sep_in,
    ali_in,
    coh_in,
    dms_filename,
    seprad_in=50,
    alirad_in=100,
    cohrad_in=150,
    initial_positions=None,
    control_callback=None,
    nframes=999,
    animate=True
):
    global sep, ali, coh, seprad, alirad, cohrad, filename, trace, flock,   \
        collection, ctrl_callback
    
    n = num_boids
    filename = dms_filename
    sep = sep_in
    ali = ali_in
    coh = coh_in
    seprad = seprad_in
    alirad = alirad_in
    cohrad = cohrad_in
    ctrl_callback = control_callback

    if os.path.exists(filename):
        os.remove(filename)
    
    # Not saving the animation so ignore this check...
    # if os.path.exists(filename+"_animation.mp4"):
    #     os.remove(filename+"_animation.mp4")

    with open(filename, 'w') as outfile:
        outfile.write(str(n) + '\n')
    width, height = 500, 250
    
    flock = Flock(n)
    if not initial_positions is None:
        if initial_positions.shape[0] != num_boids:
            print("initial_positions.shape[0] != num_boids : {} != {}".format(
                initial_positions.shape[0], num_boids
            ))
            sys.exit()
        flock.set_positions(initial_positions)
    
    collection = MarkerCollection(n)
    trace = None

    fig = plt.figure(figsize=(10, 10*height/width), facecolor="white")
    ax = fig.add_axes([0.0, 0.0, 1.0, 1.0], aspect=1, frameon=False)
    ax.add_collection(collection._collection)
    ax.set_xlim(0, width)
    ax.set_ylim(0, height)
    ax.set_xticks([])
    ax.set_yticks([])


    # animation = FuncAnimation(fig, update, interval=50, frames=999, repeat=False)
    if animate:
        animation = FuncAnimation(
            fig, update, interval=50, frames=nframes, repeat=False
        )
        plt.show()
        # Saving the animation causes update to be called again 'frames' number 
        # of times. Dont want this so comment out here. Dont really need the 
        # animation anyway...
        # animation.save(filename + '_animation.mp4', fps=20)
        # print("Animation saved")

    else:
        for _ in range(nframes):
            update()

    print("Done")

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    if len(sys.argv) == 6:
        n = int(sys.argv[1])
        sep = float(sys.argv[2])
        ali = float(sys.argv[3])
        coh = float(sys.argv[4])
        seprad = 50
        alirad = 100
        cohrad = 150
        filename = str(sys.argv[5])

    elif len(sys.argv) == 9:
        n = int(sys.argv[1])
        sep = float(sys.argv[2])
        seprad = float(sys.argv[3])
        ali = float(sys.argv[4])
        alirad = float(sys.argv[5])
        coh = float(sys.argv[6])
        cohrad = float(sys.argv[7])
        filename = str(sys.argv[8])
    else:
        print('Please input either 5 or 8 arguments: (to allow different input types, alter code at top of main)')
        print('number of points, separation, alignment, cohesion, outfile')
        print('number of points, separation, sep radius, alignment, ali radius, cohesion, coh radius, outfile')
        exit()

    run_sim(
        n,
        sep,
        ali,
        coh,
        filename,
        seprad_in=50,
        alirad_in=100,
        cohrad_in=150,
        animate=True
    )