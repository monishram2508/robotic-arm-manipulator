import numpy as np


class APFPlanner:

    def __init__(self):

        self.k_att = 1.0
        self.k_rep = 0.8
        self.repulsive_radius = 0.3

    def attractive_force(self, current, goal):

        return self.k_att * (goal - current)

    def repulsive_force(self, current, obstacles):

        force = np.zeros(3)

        for obs in obstacles:

            diff = current - obs
            dist = np.linalg.norm(diff)

            # avoid divide-by-zero
            if dist < 1e-6:
                continue

            if dist < self.repulsive_radius:

                rep = self.k_rep * (1.0/dist - 1.0/self.repulsive_radius) \
                      * (1.0/(dist**3)) * diff

                force += rep

        return force

    def compute_force(self, current, goal, obstacles):

        f_att = self.attractive_force(current, goal)
        f_rep = self.repulsive_force(current, obstacles)

        return f_att + f_rep