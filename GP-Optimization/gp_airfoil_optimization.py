import os
import subprocess
import numpy as np
from GPyOpt.methods import BayesianOptimization

def xfoil_wrapper(airfoil_file, output_file, alpha_range, iteration):
    with open('input_file.in', 'w') as f:
        f.write(f"LOAD {airfoil_file}\n")
        f.write("OPER\n")
        f.write("PACC\n")
        f.write(f"{output_file}_{iteration}\n\n")
        f.write("ITER 100\n")
        f.write(f"ASEQ {alpha_range[0]} {alpha_range[1]} {alpha_range[2]}\n")
        f.write("\n\n")
        f.write("QUIT\n")

    with open('input_file.in', 'r') as file:
        input_content = file.read().encode('utf-8')

    subprocess.run(["xfoil.exe"], input=input_content, shell=True)
    #subprocess.run(["xfoil.exe"], input=input_content, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, shell=True)

max_iter=30

def objective_function(x):
    thickness = x[:, 0]
    camber = x[:, 1]
    
    #print(f"Iteration: {optimizer.num_acquisitions}")
    print(f"Thickness: {thickness}, Camber: {camber}")

    # Generate airfoil coordinates based on thickness and camber
    airfoil_coords = generate_airfoil_coords(thickness, camber)

    # Save airfoil coordinates to a file
    airfoil_file = "airfoil.txt"
    np.savetxt(airfoil_file, airfoil_coords, fmt="%.6f")

    # Run XFOIL simulation
    output_file = "polar_file.txt"
    alpha_range = [0, 10, 0.25]
    xfoil_wrapper(airfoil_file, output_file, alpha_range, max_iter)

    # Read polar file and extract lift-to-drag ratio
    polar_data = np.loadtxt(output_file, skiprows=12)
    lift_to_drag_ratio = polar_data[:, 1] / polar_data[:, 2]
    
    print(f"Lift-to-Drag Ratio: {lift_to_drag_ratio.max()}")

    return lift_to_drag_ratio.max()

def generate_airfoil_coords(thickness, camber):
    # Read the base airfoil coordinates from NACA0012.dat
    base_coords = np.loadtxt("n0009sm.dat")

    # Modify the coordinates based on thickness and camber
    modified_coords = base_coords.copy()
    modified_coords[:, 1] *= thickness
    modified_coords[:, 1] += camber

    return modified_coords

# Define the optimization problem
bounds = [{'name': 'thickness', 'type': 'continuous', 'domain': (0.1, 0.18)},
          {'name': 'camber', 'type': 'continuous', 'domain': (0.02, 0.05)}]

# Run the optimization
optimizer = BayesianOptimization(f=objective_function,
                                 domain=bounds,
                                 model_type='GP',
                                 acquisition_type='EI',
                                 acquisition_jitter=0.05,
                                 exact_feval=True,
                                 maximize=True)

optimizer.run_optimization(max_iter, verbosity=True)

# Print the optimal results
print("Optimal thickness and camber:", optimizer.x_opt)
print("Optimal lift-to-drag ratio:", -optimizer.fx_opt)

import matplotlib.pyplot as plt

# Plotting the convergence to show the best objective function value found so far at each iteration
optimizer.plot_convergence()

# Optionally, show more detailed plots
fig, axs = plt.subplots(2)
axs[0].plot(optimizer.Y_best, 'b-', label='Best Lift-to-Drag Ratio')
axs[0].set_title('Best Lift-to-Drag Ratio Evolution')
axs[0].set_xlabel('Number of Evaluations')
axs[0].set_ylabel('Lift-to-Drag Ratio')
axs[0].legend()

axs[1].plot(optimizer.X[:, 0], 'r-', label='Thickness')
axs[1].plot(optimizer.X[:, 1], 'g-', label='Camber')
axs[1].set_title('Parameter Evolution')
axs[1].set_xlabel('Number of Evaluations')
axs[1].set_ylabel('Parameter Value')
axs[1].legend()

plt.tight_layout()
plt.show()

# Load the airfoil coordinates from airfoil.txt
airfoil_coords = np.loadtxt('airfoil.txt')

# Extract x and y coordinates
x_coords = airfoil_coords[:, 0]
y_coords = airfoil_coords[:, 1]

# Plot the airfoil shape
plt.figure(figsize=(10, 5))
plt.plot(x_coords, y_coords, label='Optimized Airfoil')
plt.fill_between(x_coords, y_coords, alpha=0.3)  # Fill under the curve for better visibility
plt.title('Airfoil Shape')
plt.xlabel('Chord Position')
plt.ylabel('Profile Thickness')
plt.grid(True)
plt.axis('equal')  # This ensures that the aspect ratio is equal and the airfoil isn't distorted
plt.legend()
plt.show()
