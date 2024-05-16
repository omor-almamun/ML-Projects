import os
import subprocess
import numpy as np
import matplotlib.pyplot as plt
from GPyOpt.methods import BayesianOptimization

class AirfoilOptimization:
    def __init__(self):
        self.bounds = [{'name': 'thickness', 'type': 'continuous', 'domain': (0.1, 0.18)},
                       {'name': 'camber', 'type': 'continuous', 'domain': (0.02, 0.05)}]
        self.optimizer = None  # Initialize as None here

    def initialize_optimizer(self):
        self.optimizer = BayesianOptimization(f=self.objective_function,
                                              domain=self.bounds,
                                              model_type='GP',
                                              acquisition_type='EI',
                                              acquisition_jitter=0.05,
                                              exact_feval=True,
                                              maximize=False)

    def xfoil_wrapper(self, airfoil_file, output_file, alpha_range):
        with open('input_file.in', 'w') as f:
            f.write(f"LOAD {airfoil_file}\n")
            #f.write("PANE\n")
            f.write("OPER\n")
            f.write("Visc 1e6\n")
            f.write("PACC\n")
            f.write(f"{output_file}\n\n")
            f.write("ITER 100\n")
            f.write(f"ASeq {alpha_range[0]} {alpha_range[1]} {alpha_range[2]}\n")
            f.write("QUIT\n")

        subprocess.run(["xfoil.exe"], input=open('input_file.in', 'r').read(), encoding='utf-8',stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, shell=True)

    def generate_airfoil_coords(self, thickness, camber):
        base_coords = np.loadtxt("NACA0012.dat")
        modified_coords = base_coords.copy()
        modified_coords[:, 1] *= thickness
        modified_coords[:, 1] += camber
        return modified_coords

    def objective_function(self, x):
        thickness = x[:, 0]
        camber = x[:, 1]
        airfoil_coords = self.generate_airfoil_coords(thickness, camber)
        airfoil_file = "airfoil.txt"
        np.savetxt(airfoil_file, airfoil_coords, fmt="%.6f")
        
        # Handle uninitialized num_acquisitions
        iteration = getattr(self.optimizer, 'num_acquisitions', 0)
        
        output_file = f"polar_file_{iteration}.txt"
        alpha_range = [0, 10, 0.25]
        self.xfoil_wrapper(airfoil_file, output_file, alpha_range)
        polar_data = np.loadtxt(output_file, skiprows=12)
        lift_to_drag_ratio = polar_data[:, 1] / polar_data[:, 2]
        return lift_to_drag_ratio.max()
    
    def plot_airfoil(self):
        # Load airfoil data
        airfoil_data = np.loadtxt('airfoil.txt')  # This assumes 'airfoil.txt' is in the current directory
        x_coords, y_coords = airfoil_data[:, 0], airfoil_data[:, 1]

        # Create the plot for airfoil geometry
        plt.figure(figsize=(10, 5))
        plt.plot(x_coords, y_coords, label='Airfoil Profile')
        plt.fill_between(x_coords, y_coords, alpha=0.3)
        plt.axis('equal')
        plt.grid(True)
        plt.xlabel('Chord Position')
        plt.ylabel('Profile Thickness')
        plt.title('Airfoil Geometry')
        plt.legend()
        plt.show()

    def run_optimization(self):
        self.initialize_optimizer()
        self.optimizer.run_optimization(max_iter=30, verbosity=True)
        print("Optimal thickness and camber:", self.optimizer.x_opt)
        print("Optimal lift-to-drag ratio:", self.optimizer.fx_opt)
        self.optimizer.plot_convergence()
        
        # Call the plot method after optimization is complete
        self.plot_airfoil()

# Instantiate and run the optimization
if __name__ == "__main__":
    optimization = AirfoilOptimization()
    optimization.run_optimization()
