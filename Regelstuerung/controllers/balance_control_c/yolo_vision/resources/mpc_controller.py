import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

"""dummy_lines = {'center': np.array([[421, 639], [502, 542], [396, 445]]),
               'left': np.array([[244, 639], [365, 542], [392, 445]]),
               'right': np.array([[598, 639], [640, 542], [401, 445]]),
               'vertical': np.array([[319, 639], [319,   0]])}
"""


def interpolate_ideal_curve(lines, angles):
    # x, y = np.hsplit(lines["center"], 2)
    x = [lines["center"][2][0], lines["center"][1][0], lines["center"][0][0]]
    y = [lines["center"][2][1], lines["center"][1][1], lines["center"][0][1]]
    ideal_curve = CubicSpline(y, x)
    print(ideal_curve)

    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(4, 1, figsize=(5, 7))

    xnew = np.linspace(390, 510, num=10)
    ynew = ideal_curve(xnew)
    ax[0].plot(xnew, ideal_curve(xnew))
    ax[0].plot(x, y, 'o', label='data')
    ax[1].plot(xnew, ideal_curve(xnew, nu=1), '--', label='1st derivative')
    ax[2].plot(xnew, ideal_curve(xnew, nu=2), '--', label='2nd derivative')
    ax[3].plot(xnew, ideal_curve(xnew, nu=3), '--', label='3rd derivative')

    for j in range(4):
        ax[j].legend(loc='best')
    plt.tight_layout()
    plt.show()
    print("plot is done")
    return ynew, xnew


def mpc_controller():
    import numpy
    import scipy.signal
    import scipy.optimize
    import matplotlib.pyplot as plt

    G = scipy.signal.lti([1], [15, 8, 1])
    plt.plot(*G.step())
    plt.show()

    M = 10  # Control horizon
    P = 20  # Prediction horizon
    DeltaT = 1  # Sampling rate

    tcontinuous = numpy.linspace(0, P * DeltaT, 1000)  # some closely spaced time points
    tpredict = numpy.arange(0, P * DeltaT, DeltaT)  # discrete points at prediction horizon

    tau_c = 1
    r = 1 - numpy.exp(-tpredict / tau_c)
    u = numpy.ones(M)

    x0 = numpy.zeros(G.to_ss().A.shape[0])

    def extend(u):
        """We optimise the first M values of u but we need P values for prediction"""
        return numpy.concatenate([u, numpy.repeat(u[-1], P - M)])

    def prediction(u, t=tpredict, x0=x0):
        """Predict the effect of an input signal"""
        t, y, x = scipy.signal.lsim(G, u, t, X0=x0, interp=False)
        return y

    plt.plot(tpredict, prediction(extend(u)))

    def objective(u, x0=x0):
        y = prediction(extend(u))
        umag = numpy.abs(u)
        constraintpenalty = sum(umag[umag > 2])
        movepenalty = sum(numpy.abs(numpy.diff(u)))
        strongfinish = numpy.abs(y[-1] - r[-1])
        return sum((r - y) ** 2) + 0 * constraintpenalty + 0.1 * movepenalty + 0 * strongfinish

    objective(u)

    result = scipy.optimize.minimize(objective, u)
    uopt = result.x
    print(result.fun)

    ucont = extend(uopt)[((tcontinuous - 0.01) // DeltaT).astype(int)]

    def plotoutput(ucont, uopt):
        plt.figure()
        plt.plot(tcontinuous, ucont)
        plt.xlim([0, DeltaT * (P + 1)])
        plt.figure()
        plt.plot(tcontinuous, prediction(ucont, tcontinuous), label='Continuous response')
        plt.plot(tpredict, prediction(extend(uopt)), '-o', label='Optimized response')
        plt.plot(tpredict, r, label='Set point')
        plt.legend()
        plt.show()

    plotoutput(ucont, uopt)


def mpc_do():
    import numpy as np
    import do_mpc
    model_type = "continuous"
    model = do_mpc.model.Model(model_type)
    phi_1 = model.set_variable(var_type='_x', var_name='phi_1', shape=(1, 1))
    dphi = model.set_variable(var_type='_x', var_name='dphi', shape=(1, 1))
    # motor soll
    phi_m_1_set = model.set_variable(var_type='_u', var_name='phi_m_1_set')
    # motor ist
    phi_1_m = model.set_variable(var_type='_x', var_name='phi_1_m', shape=(1, 1))
    print('phi_1={}, with phi_1.shape={}'.format(phi_1, phi_1.shape))

    # Inertia/Trägheit
    theta_1 = model.set_variable('parameter', 'Theta_1')


def bicycle_model_mpc(phi):
    """
    https://dingyan89.medium.com/simple-understanding-of-kinematic-bicycle-model-81cac6420357
    reference point is rear tyre
    :return:
    """
    from numpy import sin, cos, tan
    v = 4.167  # velocity bike m/s
    delta = 0  # steering angle deg
    theta = 90  # bicycle direction deg
    len_bike = 1.5
    radius = len_bike / tan(delta)
    omega = v * radius

    # nächste koordinaten, zum nächsten intervall (dot notation Zeitabhängigkeit)
    x_dot = v * cos(theta)
    y_dot = v * sin(theta)
    theta_dot = v * tan(delta) / len_bike
    delta_dot = phi

    state = np.array([x_dot, y_dot, theta_dot, delta_dot])

    pass


def predictive_model(phi, lines):
    """
    https://dingyan89.medium.com/simple-understanding-of-kinematic-bicycle-model-81cac6420357
    reference point is rear tyre
    :return:
    """
    from numpy import sin, cos, tan
    v = 4.167  # velocity bike m/s
    delta = 0  # steering angle deg
    theta = 90  # bicycle direction deg
    len_bike = 1.5
    radius = len_bike / tan(delta)
    omega = v * radius

    # nächste koordinaten, zum nächsten intervall (dot notation Zeitabhängigkeit)
    x_dot = v * cos(theta + delta)
    y_dot = v * sin(theta + delta)
    theta_dot = v * sin(delta) / len_bike
    delta_dot = phi

    state = np.array([x_dot, y_dot, theta_dot, delta_dot])

    # with time delta:
    time_delta = 0.5  # in seconds
    x_1 = lines["center"][1][0] + x_dot * time_delta
    y_1 = lines["center"][1][1] + y_dot * time_delta
    theta_1 = theta + theta_dot * time_delta
    delta_1 = delta + delta_dot * time_delta

    state_1 = np.array([x_1, y_1, theta_1, delta_1])

    pass


def test(lines, angles):
    import numpy as np
    from numpy import sin, cos, tan

    # Extract initial positions and angles
    x = [lines["center"][2][0], lines["center"][1][0], lines["center"][0][0]]
    y = [lines["center"][2][1], lines["center"][1][1], lines["center"][0][1]]
    phi = [0, angles["center"]["vec1"][0], angles["center"]["vec2"][0]]

    # Define constants
    v = 4.167  # velocity bike m/s
    len_bike = 1.5
    dt = 0.1  # time step in seconds

    # Initial state
    states = []
    for i in range(3):
        state = np.array([x[i], y[i], np.deg2rad(90), 0])  # [x, y, theta, delta]
        states.append(state)

    # Define the kinematic bicycle model function
    def kinematic_bicycle_model(state, phi, dt):
        x, y, theta, delta = state

        x_dot = v * cos(theta)
        y_dot = v * sin(theta)
        theta_dot = v * tan(delta) / len_bike
        delta_dot = phi

        x += x_dot * dt
        y += y_dot * dt
        theta += theta_dot * dt
        delta += delta_dot * dt

        return np.array([x, y, theta, delta])

    # Simulate the model for each state using the corresponding phi
    trajectories = []
    for state, phi_i in zip(states, phi):
        trajectory = [state]
        for _ in range(10):  # Simulate for 10 time steps as an example
            state = kinematic_bicycle_model(state, phi_i, dt)
            trajectory.append(state)
        trajectories.append(np.array(trajectory))

    # Print the trajectories for each initial position
    for i, trajectory in enumerate(trajectories):
        print(f"Trajectory {i + 1}:")
        print(trajectory)
        print()

    return trajectories


# Example usage
"""lines = {
    "center": [
        [0, 0],  # point 0
        [1, 1],  # point 1
        [2, 2]  # point 2
    ]
}

angles = {
    "center": {
        "vec1": [0.1],  # angle for point 1
        "vec2": [0.2]  # angle for point 2
    }
}

trajectories = test(lines, angles)"""


def python_robotics(x, y):

    pass


if __name__ == '__main__':
    mpc_controller()


