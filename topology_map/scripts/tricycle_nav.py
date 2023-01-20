from pickle import TRUE
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import rospy
from nav_msgs.msg import Path
receive_one_msg_flag = True
def callback(msg):
    if receive_one_msg_flag:
        j=int(len(msg.poses))-1
        print ("j=",j,"  len(msg.poses)=",len(msg.poses))
        # print(msg.poses[len(msg.poses)-1].pose.position.x , msg.poses[len(msg.poses)-1].pose.position.y)
        for i in range(j):
            print("point ",i,msg.poses[i].pose.position.x , msg.poses[i].pose.position.y)
        receive_one_msg_flag = False

def listener():
    rospy.init_node('path', anonymous=True)
    rospy.Subscriber("/move_base/TebLocalPlannerROS/global_plan", Path, callback)
    rospy.spin()
    
def rdp(points, epsilon):
    # get the start and end points
    start = np.tile(np.expand_dims(points[0], axis=0), (points.shape[0], 1))
    end = np.tile(np.expand_dims(points[-1], axis=0), (points.shape[0], 1))

    # find distance from other_points to line formed by start and end
    dist_point_to_line = np.abs(np.cross(end - start, points - start, axis=-1)) / np.linalg.norm(end - start, axis=-1)
    # get the index of the points with the largest distance
    max_idx = np.argmax(dist_point_to_line)
    max_value = dist_point_to_line[max_idx]

    result = []
    if max_value > epsilon:
        partial_results_left = rdp(points[:max_idx+1], epsilon)
        result += [list(i) for i in partial_results_left if list(i) not in result]
        partial_results_right = rdp(points[max_idx:], epsilon)
        result += [list(i) for i in partial_results_right if list(i) not in result]
    else:
        result += [points[0], points[-1]]

    return result


if __name__ == "__main__":
    min_x = -1
    max_x = 10
    # listener()
    xs = np.linspace(min_x, max_x, num=200)
    # ys = np.cos(2 * np.pi * xs)
    ys = np.exp(-xs) * np.cos(2 * np.pi * xs)
    print(xs)
    print(ys)
    sample_points = np.concatenate([
        np.expand_dims(xs, axis=-1),
        np.expand_dims(ys, axis=-1)
    ], axis=-1)
    # print(sample_points)
    # First set up the figure, the axis, and the plot element we want to animate
    fig = plt.figure()
    ax = plt.axes(xlim=(min_x, max_x), ylim=(-1, 1))
    plt.xlabel("x")
    plt.ylabel("y")
    text_values = ax.text(
        0.70,
        0.15,
        "",
        transform=ax.transAxes,
        fontsize=12,
        verticalalignment='top',
        bbox=dict(boxstyle='round',
                  facecolor='wheat',
                  alpha=0.2)
    )
    original_line, = ax.plot(xs, ys, lw=2, label=r"$y = e^{-x}cos(2 \pi x)$")
    simplified_line, = ax.plot([], [], lw=2, label="simplified", marker='o', color='r')

    # initialization function: plot the background of each frame
    def init():
        simplified_line.set_data(xs, ys)
        return original_line, simplified_line, text_values

    # animation function.  This is called sequentially
    def animate(i):
        epsilon = 0 + (i * 0.1)
        simplified = np.array(rdp(sample_points, epsilon))
        print("i: {i}, episilon: {'%.1f' % epsilon}, n: {simplified.shape[0]}")
        simplified_line.set_data(simplified[:, 0], simplified[:, 1])
        text_values.set_text("$\epsilon$: {'%.1f' % epsilon}, $n$: {simplified.shape[0]}")
        return original_line, simplified_line, text_values

    # call the animator.  blit=True means only re-draw the parts that have changed.
    anim = animation.FuncAnimation(
        fig,
        animate,
        init_func=init,
        frames=21,
        interval=1000,
        repeat=True
    )
    plt.legend()
    plt.show()
    