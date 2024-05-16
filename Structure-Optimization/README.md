# Structure Optimization

This project involves topology optimization for a load-bearing cantilever structure.

## Objective

The objective is to determine the optimal topology of a cantilever beam that can carry a specified load.

## Problem Description

The initial workspace is a rectangular bar with a load \( P \) applied at the end. The structure is discretized into a fine rectangular grid of nodes. Nodes connected using the nearest neighbor algorithm define the members. The optimization problem aims to minimize the structure's mass while ensuring it can carry the load \( P \).

## Files

- `structure_optimization.py`: Main script for performing the topology optimization.

## Instructions

1. **Discretize the Structure**:
   - The structure is discretized into a fine rectangular grid with nodes equally spaced by `dx` and `dy`.
   - Only nodes attached to the left fixture have reaction forces; other nodes are free.

2. **Generate Connectivity Matrix**:
   - Use the nearest neighbor algorithm to connect nodes within a distance of \( \sqrt{dx^2 + dy^2} \).
   - Ensure no duplicate members in the connectivity matrix.

3. **Solve the Optimization Problem**:
   - Formulate the problem to minimize the mass of the structure, considering equilibrium conditions.
   - The structure's mass is equivalent to the weighted 1-norm of the force densities.

4. **Compute Member Dimensions**:
   - Choose a material and compute the minimum radius for each member based on yield stress (tensile members) and buckling stress (compressive members).
   - Calculate the total mass of the structure from this information.

## Dependencies

- Ensure you have the necessary Python libraries installed. You can use `pip` to install any missing dependencies.

## Results

The optimized structure should resemble a typical cantilever beam optimized for minimum mass, similar to the example provided in the assignment.

## Comments

This project involves solving a complex optimization problem with practical applications in structural engineering. The solution methodology includes discretization, connectivity matrix generation, optimization problem formulation, and computation of member dimensions based on material properties.
