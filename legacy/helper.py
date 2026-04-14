# build18 - hexapod bot 
# does the math for a yaw-roll-roll leg (x6) 

import math 
import numpy as np

class Robot:
    def __init__(self):
        """Initialize robot model"""
        self.dof = 3

        # 3-DOF yaw-roll-roll (Z-X-X)
        # DH row format (matching your Franka code): [a, alpha, d, theta]
        #
        # A common standard-DH choice:
        #   1) yaw about z, then transition to x for roll joints -> alpha1 = +pi/2
        #   2) roll about x -> alpha2 = 0
        #   3) roll about x -> alpha3 = 0
        #
        # Replace a2, a3, d1 with your actual geometry numbers.

        d1 = 0.5   # base offset along z (example)
        a2 = 1.0   # link 2 length (example)
        a3 = 1.0   # link 3 length (example)
        
        # the order goes [a, α, d, θ]
        
        # reference configuration (a.k.a home position sets theta = 0)

        self.dh_parameters = np.array([
            [0.0,   np.pi/2,  d1,  0.0],  # joint 1: yaw
            [a2,    0.0,        0.0, 0.0],  # joint 2: roll
            [a3,    0.0,        0.0, 0.0],  # joint 3: roll
        ])


    def forward_kinematics(self, dh_parameters, thetas):
        """
        Forward kinematics for a 3-DOF yaw-roll-roll arm using the same DH row format you used:
            [a, alpha, d, theta_offset]

        Uses the SAME transform composition order as your original code:
            H_i = T_x(a) @ R_x(alpha) @ R_z(theta) @ T_z(d)
        (i.e., the order you labeled 'modified DH' in your comments)
        """
        thetas = np.asarray(thetas, dtype=float)

        if thetas.ndim != 1:
            raise ValueError("Expecting a 1D array of joint angles.")
        if thetas.shape[0] != self.dof:
            raise ValueError(f"Invalid number of joints: {thetas.shape[0]} found, expecting {self.dof}")

        dh = np.asarray(dh_parameters, dtype=float)
        if dh.shape != (self.dof, 4):
            raise ValueError(f"Expecting dh_parameters shape ({self.dof}, 4) = ({self.dof}, 4), got {dh.shape}")

        H = np.eye(4)
        frames = np.zeros((4, 4, self.dof + 1))
        frames[:, :, 0] = np.eye(4)

        for i in range(self.dof):
            a_i, alpha_i, d_i, theta_off = dh[i, 0], dh[i, 1], dh[i, 2], dh[i, 3]
            theta_i = thetas[i] + theta_off  # yaw (joint 1), roll (joint 2), roll (joint 3)

            ca, sa = np.cos(alpha_i), np.sin(alpha_i)
            ct, st = np.cos(theta_i), np.sin(theta_i)

            # T_x(a)
            T_x = np.array([
                [1, 0, 0, a_i],
                [0, 1, 0, 0.0],
                [0, 0, 1, 0.0],
                [0, 0, 0, 1.0]
            ])

            # R_x(alpha)
            R_x = np.array([
                [1, 0,  0, 0.0],
                [0, ca, -sa, 0.0],
                [0, sa,  ca, 0.0],
                [0, 0,  0, 1.0]
            ])

            # R_z(theta)
            R_z = np.array([
                [ct, -st, 0, 0.0],
                [st,  ct, 0, 0.0],
                [0.0, 0.0, 1, 0.0],
                [0.0, 0.0, 0, 1.0]
            ])

            # T_z(d)
            T_z = np.array([
                [1, 0, 0, 0.0],
                [0, 1, 0, 0.0],
                [0, 0, 1, d_i],
                [0, 0, 0, 1.0]
            ])

            H_i = T_x @ R_x @ R_z @ T_z
            H = H @ H_i
            frames[:, :, i + 1] = H

        return H
    

def compute_jacobian_analytical(self, thetas):
    """
    Compute the Jacobian for the given joint configuration for a 3-DOF yaw-roll-roll arm.

    Parameters
    ----------
    thetas : np.ndarray
        Joint configuration (3,)

    Returns
    -------
    J : np.ndarray
        6x3 Jacobian matrix [linear; angular]
    """
    thetas = np.asarray(thetas, dtype=float)

    if thetas.ndim != 1:
        raise ValueError("Expecting a 1D array of joint angles.")
    if thetas.shape[0] != self.dof:
        raise ValueError(f"Expected {self.dof} joint angles, got {thetas.shape[0]}")

    dh = np.asarray(self.dh_parameters, dtype=float)
    n = self.dof  # = 3

    # origins p_i and z-axes z_i in base frame, for frames 0..n
    origins = np.zeros((3, n + 1))   # p_0 ... p_n
    z_axes  = np.zeros((3, n + 1))   # z_0 ... z_n

    # base frame
    H = np.eye(4)
    origins[:, 0] = H[0:3, 3]  # [0,0,0]
    z_axes[:, 0]  = H[0:3, 2]  # [0,0,1]

    # forward pass: build T_0i and store p_i, z_i (frame i after joint i transform)
    for i in range(n):
        a_i, alpha_i, d_i, theta_off = dh[i, 0], dh[i, 1], dh[i, 2], dh[i, 3]
        theta_i = thetas[i] + theta_off

        ca, sa = np.cos(alpha_i), np.sin(alpha_i)
        ct, st = np.cos(theta_i), np.sin(theta_i)

        T_x = np.array([
            [1, 0, 0, a_i],
            [0, 1, 0, 0.0],
            [0, 0, 1, 0.0],
            [0, 0, 0, 1.0]
        ])

        R_x = np.array([
            [1, 0,  0, 0.0],
            [0, ca, -sa, 0.0],
            [0, sa,  ca, 0.0],
            [0, 0,  0, 1.0]
        ])

        R_z = np.array([
            [ct, -st, 0.0, 0.0],
            [st,  ct, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])

        T_z = np.array([
            [1, 0, 0, 0.0],
            [0, 1, 0, 0.0],
            [0, 0, 1, d_i],
            [0, 0, 0, 1.0]
        ])

        H_i = T_x @ R_x @ R_z @ T_z
        H = H @ H_i

        origins[:, i + 1] = H[0:3, 3]
        z_axes[:, i + 1]  = H[0:3, 2]

    p_n = origins[:, n]

    # Jacobian (geometric form): [v; w]
    J = np.zeros((6, n))

    for i in range(n):
        # IMPORTANT: joint i axis is z_{i} (not z_{i+1}),
        # and joint i origin is p_{i}
        z_i = z_axes[:, i]
        p_i = origins[:, i]

        Jv_i = np.cross(z_i, (p_n - p_i))
        Jw_i = z_i

        J[0:3, i] = Jv_i
        J[3:6, i] = Jw_i

    return J


def inverse_kinematics(
        self,
        target_pos,
        q0=None,
        max_iters=200,
        tol=1e-4,
        damping=1e-3,
        step_size=1.0,
        joint_limits=None,
    ):
        """
        Numerical IK for the 3-DOF yaw-roll-roll arm to reach a TARGET POSITION.

        Because the robot has only 3 DOF, it generally cannot satisfy a full 6D pose constraint.
        This solver targets end-effector position only (x, y, z).

        Parameters
        ----------
        target_pos : array-like, shape (3,)
            Desired end-effector position in base frame.
        q0 : array-like, shape (3,), optional
            Initial guess. Defaults to zeros.
        max_iters : int
            Maximum iterations.
        tol : float
            Convergence tolerance on position error norm.
        damping : float
            Damped least squares lambda (stabilizes near singularities).
        step_size : float
            Step scaling (1.0 is typical).
        joint_limits : tuple(np.ndarray, np.ndarray) or None
            Optional (q_min, q_max), each shape (3,). If provided, solution is clamped.

        Returns
        -------
        q : np.ndarray, shape (3,)
            Joint angles solution.
        success : bool
            True if converged within tol, else False.
        """
        target_pos = np.asarray(target_pos, dtype=float).reshape(3)
        if q0 is None:
            q = np.zeros(self.dof, dtype=float)
        else:
            q = np.asarray(q0, dtype=float).reshape(self.dof)

        if joint_limits is not None:
            q_min, q_max = joint_limits
            q_min = np.asarray(q_min, dtype=float).reshape(self.dof)
            q_max = np.asarray(q_max, dtype=float).reshape(self.dof)

        # DLS uses Jv (3x3) for position-only
        I3 = np.eye(3)

        for _ in range(max_iters):
            # FK -> current end-effector position
            H, = (self.forward_kinematics(self.dh_parameters, q),)  # keep your FK signature
            p = H[0:3, 3]

            # position error
            e = target_pos - p
            if np.linalg.norm(e) < tol:
                return q, True

            # Jacobian (geometric). Take only linear part (top 3 rows)
            J = self.compute_jacobian_analytical(q)   # returns 6x3 [v; w]
            Jv = J[0:3, :]                            # 3x3

            # Damped least squares step:
            # dq = Jv^T (Jv Jv^T + λ^2 I)^(-1) e
            A = Jv @ Jv.T + (damping ** 2) * I3
            dq = Jv.T @ np.linalg.solve(A, e)

            q = q + step_size * dq

            # optional clamp
            if joint_limits is not None:
                q = np.clip(q, q_min, q_max)

        return q, False
