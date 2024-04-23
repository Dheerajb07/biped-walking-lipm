import numpy as np

def step_params(step_len, step_width, NSteps):
    N = NSteps + 2          # Number of intervals
    # Initialize arrays
    sx = np.zeros(N)
    sy = np.zeros(N)
    
    # Loop through steps
    for i in range(1, N+1):
        if i == 1:
            sx[i-1] = 0
            sy[i-1] = step_width / 2
        elif i < N:
            sx[i-1] = step_len
            sy[i-1] = step_width
        else:
            sx[i-1] = 0
            sy[i-1] = step_width
    
    return sx, sy

def cubicpolytraj(keyframes, tspan, timevec):
    # keyframes: Matrix of keyframe positions (each column corresponds to a dimension)
    # tspan: Time span of the trajectory
    # timevec: Time vector for evaluating the trajectory

    # Number of dimensions
    dim = keyframes.shape[0]

    # Initialize coefficients matrix
    coeffs = np.zeros((dim, 4))

    # Construct coefficient matrix for each dimension
    for d in range(dim):
        # Formulate and solve system of equations for each dimension
        A = np.array([[tspan[0] ** 3, tspan[0] ** 2, tspan[0], 1],
                      [3 * tspan[0] ** 2, 2 * tspan[0], 1, 0],
                      [tspan[1] ** 3, tspan[1] ** 2, tspan[1], 1],
                      [3 * tspan[1] ** 2, 2 * tspan[1], 1, 0]])
        b = np.array([keyframes[d, 0], 0, keyframes[d, 1], 0])

        # Solve for coefficients
        coeffs[d] = np.linalg.solve(A, b)

    # Evaluate trajectory at timevec
    q = np.zeros((dim, len(timevec)))
    qd = np.zeros((dim, len(timevec)))
    qdd = np.zeros((dim, len(timevec)))

    for i, t in enumerate(timevec):
        if t < tspan[0]:
            q[:, i] = keyframes[:, 0]
            qd[:, i] = np.zeros(dim)
            qdd[:, i] = np.zeros(dim)
        elif t > tspan[1]:
            q[:, i] = keyframes[:, 1]
            qd[:, i] = np.zeros(dim)
            qdd[:, i] = np.zeros(dim)
        else:
            for d in range(dim):
                q[d, i] = coeffs[d, 0] * t ** 3 + coeffs[d, 1] * t ** 2 + coeffs[d, 2] * t + coeffs[d, 3]
                qd[d, i] = 3 * coeffs[d, 0] * t ** 2 + 2 * coeffs[d, 1] * t + coeffs[d, 2]
                qdd[d, i] = 6 * coeffs[d, 0] * t + 2 * coeffs[d, 1]

    return q, qd, qdd


def getKneeBendingTraj(init_pos, final_pos, tspan, dt):
    timevec = np.arange(tspan[0], tspan[1] + dt, dt)
    q, qd, qdd = cubicpolytraj(np.column_stack((init_pos, final_pos)), tspan, timevec)
    comPos = {'x': q[0, :], 'y': q[1, :], 'z': q[2, :]}
    comVel = {'x': qd[0, :], 'y': qd[1, :], 'z': qd[2, :]}
    return comPos, comVel

def getSwingFootTraj(footpos0, footpos1, swingheight, timestp0, timestpf, Ts):
    # Trajectory for X and Y
    waypointsXY = np.vstack((footpos0[:2], footpos1[:2]))
    timestampsXY = [timestp0, timestpf]
    timevecswingXY = np.arange(timestampsXY[0], timestampsXY[1] + Ts, Ts)
    XYq, XYqd, XYqdd = cubicpolytraj(waypointsXY, timestampsXY, timevecswingXY)

    # Trajectory for Z
    waypointsZ = [footpos0[2], footpos0[2] + swingheight, footpos0[2]]
    timestpMid = (timestp0 + timestpf) / 2  # Top of swing at midpoint
    timestampsZ = [timestp0, timestpMid, timestpf]
    timevecswingZ = np.arange(timestampsZ[0], timestampsZ[2] + Ts, Ts)
    Zq, Zqd, Zqdd = cubicpolytraj(np.array([waypointsZ]).T, timestampsZ, timevecswingZ)

    # Combine xy and z trajectory
    q = np.vstack((XYq, Zq))
    qd = np.vstack((XYqd, Zqd))
    qdd = np.vstack((XYqdd, Zqdd))

    return q, qd, qdd
