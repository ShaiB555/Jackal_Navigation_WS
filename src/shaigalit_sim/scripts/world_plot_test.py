import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import rospy


def remove_first_column(matrix):
    return [row[1:] for row in matrix]

if __name__ == '__main__':
    # Create a figure and axis
    fig, ax = plt.subplots()
    plt.grid(which='both')  # Turn off grid lines (optional)

    # Set the background color to gray
    ax.set_facecolor('white')

    # Define circle parameters (position, radius, and color)
    circle_params = [(0,0,0.15, 'green'), (6,6,0.25, 'green'), (-6,-6,0.31, 'green'), (-6,6,0.46, 'green'), (6,-6,0.54, 'green')]

    # Plot circles
    for x, y, r, color in circle_params:
        circle = plt.Circle((x, y), r, color=color, fill=True)  # Fill with color
        ax.add_patch(circle)

    # Define square parameters (position, size, and color)
    square_params = [(0,5, 1, 'red'), (7,0,1, 'red'), (0,-5,1, 'red'), (-7,0,1, 'red'), (-2,1,0.55, 'red')]

    # Plot squares
    for x, y, size, color in square_params:
        square = patches.Rectangle((x - size / 2, y - size / 2), size, size, linewidth=1, edgecolor='black', facecolor=color)
        ax.add_patch(square)

    # # Set axis limits
    ax.set_xlim(-8, 8)
    ax.set_ylim(-8, 8)

    # Set axis labels (optional)
    ax.set_xlabel('X Position [m]')
    ax.set_ylabel('Y Position [m]')

    # Set plot title (optional)
    plt.title('Navigation Plot')

    # Show the plot
    plt.gca().set_aspect('equal', adjustable='box')  # Equal aspect ratio

    x_est_vec=rospy.get_param("x_est_vec",[[0.0],[0.0],[0.0]])
    x_est_vec=remove_first_column(x_est_vec)
    x_est_vec=np.array(x_est_vec)
    plt.plot(x_est_vec[0],x_est_vec[1],"b-")
    plt.plot(x_est_vec[0,0],x_est_vec[1,0],"c>",MarkerSize=10)
    plt.plot(x_est_vec[0,-1],x_est_vec[1,-1],"yX",MarkerSize=10)
    plt.show()
