import rospy
import numpy as np
import angle_diff

# # Initialize the ROS node
# rospy.init_node('param_example')

# # Retrieve the value of a parameter named 'my_parameter'
# param_value = rospy.get_param('at_goal')

# # Print the parameter value
# print("Parameter value:", param_value)

# alpha=1*np.pi
# beta=(1/4)*np.pi
# print(angle_diff.angle_diff(alpha,beta))

# print(np.eye(3))

import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

# Define your circle function
def circ(pos, x0, y0, R):
    return (pos[0] - x0)**2 + (pos[1] - y0)**2 - R**2

# Generate example data points on a circle
theta = np.linspace(0, 2*np.pi, 100)
circle_x = 3 + 2 * np.cos(theta)
circle_y = 2 + 2 * np.sin(theta)

# Adding some random noise to simulate measurement errors
noise = np.random.normal(0, 0.1, len(theta))
noisy_x = circle_x + noise
noisy_y = circle_y + noise

# Perform the curve fit
initial_guess = [0, 0, 1]  # Initial guess for circle parameters
params, covariance = curve_fit(circ, (noisy_x, noisy_y), np.zeros_like(noisy_x), p0=initial_guess)

# Extract the fitted parameters
x0_fit, y0_fit, R_fit = params

# Calculate the residuals between the fitted circle and noisy data
residuals = circ((noisy_x, noisy_y), x0_fit, y0_fit, R_fit)

# Calculate the total sum of squares (TSS) from the noisy data
tss = np.sum((noisy_x - np.mean(noisy_x))**2 + (noisy_y - np.mean(noisy_y))**2)

# Calculate the sum of squared residuals (RSS) from the circle fit
rss = np.sum(residuals**2)

# Calculate the pseudo-R-squared value
pseudo_r_squared = 1 - (rss / tss)

print("Pseudo-R-squared:", pseudo_r_squared)

# Create a figure and plot the data points
plt.figure()
plt.scatter(noisy_x, noisy_y, label='Noisy Data')

# Plot the fitted circle using the fitted parameters
theta_fit = np.linspace(0, 2*np.pi, 100)
fit_x = x0_fit + R_fit * np.cos(theta_fit)
fit_y = y0_fit + R_fit * np.sin(theta_fit)
plt.plot(fit_x, fit_y, label='Fitted Circle', color='red')

plt.xlabel('X')
plt.ylabel('Y')
plt.title('Circle Fitting')
plt.legend()
plt.grid()
plt.axis('equal')
plt.show()
