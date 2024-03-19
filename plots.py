# This code is GPT-generated

import matplotlib.pyplot as plt
import numpy as np

# Data
samples = [10, 100, 1000, 10000, 100000]
mean_times = [3.8536e-05, 0.00019742, 0.005309142, 0.3331366, 36.18164]  # Mean time taken to plan
std_times = [1.65586e-05, 5.74293e-05, 1.84995e-03, 9.34635e-03, 7.06743e-01]  # Standard deviations of time taken to plan
path_costs_before = [2.51346, 2.51346, 2.95102, 3.06365, 3.00525]
path_costs_after = [1.9628, 1.9628, 1.9628, 1.9628, 1.9628]  # New costs after shortcutting
std_path_costs_before = [0, 0, np.std([2.95102, 3.05501, 3.05501, 3.32315, 3.32315]), 
                         np.std([3.06365, 3.03754, 3.33815, 3.33815, 2.94719]), 
                         np.std([3.00525, 3.04383, 2.9712, 2.90395, 3.1811])]
std_path_costs_after = [0, 0, 0, 0, 0]  # Assuming no variation after shortcutting

# Plot
plt.figure(figsize=(12, 6))

# Time taken to plan plot
plt.subplot(1, 2, 1)
plt.errorbar(samples, mean_times, yerr=std_times, fmt='o-', label='Mean Time Taken to Plan')
plt.xscale('log')
plt.yscale('log')
plt.xlabel('Number of Samples')
plt.ylabel('Time (seconds)')
plt.title('Mean Time Taken to Plan vs Number of Samples')
plt.grid(True)
plt.legend()

# Path cost plot with shortcutting
plt.subplot(1, 2, 2)
plt.errorbar(samples, path_costs_before, yerr=std_path_costs_before, fmt='o-', label='Path Cost Before Shortcutting')
plt.errorbar(samples, path_costs_after, yerr=std_path_costs_after, fmt='o-', label='Path Cost After Shortcutting')
plt.xscale('log')
plt.yscale('log')
plt.xlabel('Number of Samples')
plt.ylabel('Path Cost')
plt.title('Path Cost Before and After Shortcutting vs Number of Samples')
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()
