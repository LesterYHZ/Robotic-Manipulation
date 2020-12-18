import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


def render(traj):
    fig = plt.figure(figsize=(5, 5))
    ax = fig.add_subplot(111, autoscale_on=False, xlim=(-2, 2), ylim=(-0.1, 2))
    ax.set_aspect('equal')
    ax.grid()

    line, = ax.plot([], [], 'o-', lw=2)
    time_template = 'time = %.1fs'
    time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)
    dt = 0.01

    def animate(i):
        # draws square given its configuration
        local_corners = np.array([[-0.2, -0.2, 0.2, 0.2, -0.2],
                                  [-0.2, 0.2, 0.2, -0.2, -0.2],
                                  [1, 1, 1, 1, 1]])
        H = np.array([[np.cos(traj[2, i]), -np.sin(traj[2, i]), traj[0, i]],
                      [np.sin(traj[2, i]), np.cos(traj[2, i]), traj[1, i]],
                      [0., 0., 1]])
        world_corners = H @ local_corners

        line.set_data(world_corners[0, :], world_corners[1, :])
        time_text.set_text(time_template % (i * dt))
        return line, time_text

    ani = animation.FuncAnimation(
        fig, animate, traj.shape[1], interval=dt * 3000, blit=True)
    plt.show()


if __name__ == "__main__":
    render()