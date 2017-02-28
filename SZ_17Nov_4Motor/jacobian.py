import numpy as np
import numpy.matlib
from scipy.linalg import block_diag
from numpy.linalg import pinv

"""
Compute the Jacobian Matrix given the configuration of the robot
"""


class jacobianMatrix:

    def __init__(self, point_matrix, attach_points):
        "initialize the class"
        """
        point_matrix: [x1, y1, z1; x2, y2, z2; ...]
        attach_points: [x1, y1, z1, type; x2, y2, z2, type; ...]
        """
        self.point_matrix = point_matrix
        self.attach_points = attach_points

        self.attach_points_tether = self.attach_points[
            self.attach_points[:, 3] == 0, :]
        self.attach_points_tube = self.attach_points[
            self.attach_points[:, 3] == 1, :]

    def cal_L(self):
        """
        Calculate L matrix, which is N * M * 3, N is the number of
        points and M is number of attach points
        l_i = a_i - p
        """
        # calculate the l matrix
        self.point_matrixs = self.point_matrix.reshape(
            self.point_matrix.shape[0], 1, self.point_matrix.shape[1])
        self.point_matrixs = np.tile(self.point_matrixs,
                                     (self.attach_points.shape[0], 1))
        self.attach_points_matrix = np.matlib.repmat(
            self.attach_points[:, 0:3], self.point_matrix.shape[0], 1)
        self.attach_points_matrix = self.attach_points_matrix.reshape(
            self.point_matrix.shape[0], self.attach_points.shape[0], 3)
        self.L = np.subtract(self.attach_points_matrix,
                             self.point_matrixs)
        # self.L[:,self.attach_points[:,3]==1,:] = \
        #                     - self.L[:,self.attach_points[:,3]==1,:]
        # print(self.L)

    def cal_B(self):
        """
        Compute B matrix, which makes B * dL = A * dp
        """
        self.B = np.zeros((self.point_matrix.shape[0],
                           self.attach_points.shape[0],
                           self.point_matrix.shape[1]
                           * self.attach_points.shape[0]))
        for i in range(0, self.point_matrix.shape[0]):
            self.B[i, :, :] = block_diag(* self.L[i, :, :])
        self.L_tether = self.L[:, self.attach_points[:, 3] == 0, :]
        self.L_tube = self.L[:, self.attach_points[:, 3] == 1, :]

        self.B_tether = np.zeros((self.point_matrix.shape[0],
                                  self.attach_points_tether.shape[0],
                                  self.point_matrix.shape[1]
                                  * self.attach_points_tether.shape[0]))
        for i in range(0, self.point_matrix.shape[0]):
            self.B_tether[i, :, :] = block_diag(* self.L_tether[i, :, :])

        self.B_tube = np.zeros((self.point_matrix.shape[0],
                                self.attach_points_tube.shape[0],
                                self.point_matrix.shape[1]
                                * self.attach_points_tube.shape[0]))
        for i in range(0, self.point_matrix.shape[0]):
            self.B_tube[i, :, :] = block_diag(* self.L_tube[i, :, :])

    def cal_A(self):
        """
        Compute A matrix, which makes B * dL = A * dp
        """
        self.A = -self.L
        self.A_tether = self.A[:, self.attach_points[:, 3] == 0, :]
        self.A_tube = self.A[:, self.attach_points[:, 3] == 1, :]
        # A = self.L
        # self.A = A
        # print(self.A)
        # print(self.L)
        # self.A[:,self.attach_points[:,3]==1,:] = \
        #                         - A[:,self.attach_points[:,3]==1,:]
        # print(self.A)
        # print(self.L)

    def cal_J_BA(self):
        """
        Compute Jacobian Matrix, J = B^-1 * A, F = J^T * f
        """
        self.cal_L()
        self.cal_B()
        self.cal_A()

        self.B_plus = np.zeros((self.point_matrix.shape[0],
                                self.point_matrix.shape[1]
                                * self.attach_points.shape[0],
                                self.attach_points.shape[0]))
        self.J_BA = np.zeros((self.point_matrix.shape[0],
                              self.point_matrix.shape[1]
                              * self.attach_points.shape[0],
                              self.point_matrix.shape[1]))
        for i in range(0, self.point_matrix.shape[0]):
            self.B_plus[i, :, :] = pinv(self.B[i, :, :])
            self.J_BA[i, :, :] = np.dot(self.B_plus[i, :, :], self.A[i, :, :])

    def cal_J_AB(self):
        """
        Compute Jacobian Matrix, J = A^-1 * B
        """
        self.cal_L()
        self.cal_B()
        self.cal_A()

        self.A_plus = np.zeros((self.point_matrix.shape[0],
                                self.point_matrix.shape[1],
                                self.attach_points.shape[0]))
        self.J_AB = np.zeros((self.point_matrix.shape[0],
                              self.point_matrix.shape[1],
                              self.point_matrix.shape[1]
                              * self.attach_points.shape[0]))
        for i in range(0, self.point_matrix.shape[0]):
            self.A_plus[i, :, :] = pinv(self.A[i, :, :])
            self.J_AB[i, :, :] = np.dot(self.A_plus[i, :, :], self.B[i, :, :])

    def cal_tether_J(self):
        """
        Compute Jacobian for tethers
        """

        self.B_tether_plus = np.zeros((self.point_matrix.shape[0],
                                       self.point_matrix.shape[1]
                                       * self.attach_points_tether.shape[0],
                                       self.attach_points_tether.shape[0]))
        self.J_tether = np.zeros((self.point_matrix.shape[0],
                                  self.point_matrix.shape[1]
                                  * self.attach_points_tether.shape[0],
                                  self.point_matrix.shape[1]))
        for i in range(0, self.point_matrix.shape[0]):
            self.B_tether_plus[i, :, :] = pinv(self.B_tether[i, :, :])
            self.J_tether[i, :, :] = np.dot(self.B_tether_plus[i, :, :],
                                            self.A_tether[i, :, :])

    def cal_tube_J(self):
        """
        Compute Jacobian for tube
        """
        self.B_tube_plus = np.zeros((self.point_matrix.shape[0],
                                     self.point_matrix.shape[1]
                                     * self.attach_points_tube.shape[0],
                                     self.attach_points_tube.shape[0]))
        self.J_tube = np.zeros((self.point_matrix.shape[0],
                                self.point_matrix.shape[1]
                                * self.attach_points_tube.shape[0],
                                self.point_matrix.shape[1]))
        for i in range(0, self.point_matrix.shape[0]):
            self.B_tube_plus[i, :, :] = pinv(self.B_tube[i, :, :])
            self.J_tube[i, :, :] = np.dot(
                self.B_tube_plus[i, :, :], self.A_tube[i, :, :])

if __name__ == '__main__':
    #    point_matrix = np.array([[1,1,1],[2,2,2]])
    point_matrix = np.array([[2, 2.5, 2.5], [2.5, 2.5, 2.5], [3, 2.5, 2.5]])
    attach_points = np.array([[0, 1, 4, 0], [0, 2.5, 1, 0], [
                             0, 4, 4, 0], [0, 2.5, 2.5, 1]])
    # point_matrix = np.array([[0,3,2]])
    # attach_points = np.array([[0,0,0, 0],[0,4,0, 0],[0,2.5,0, 1]])
    J = jacobianMatrix(point_matrix, attach_points)
    # J.cal_L()
    # J.cal_B()
    # J.cal_A()
    J.cal_J_AB()
