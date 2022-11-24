# Plot Runtime (s) vs. Number of Agents
# for the 2 different algorithms

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# Read in data
data_1 = pd.read_csv('pto_simulation_results.csv')
data_2 = pd.read_csv('coupled_simulation_results.csv')

# Data - PTO
agents = data_1.iloc[:, 0]    # Number of agents
prioritized_planning_runtime = data_1.iloc[:, -2]   # Prioritized Planning Runtime
prioritized_planning_cost = data_1.iloc[:, -1]  # Prioritized Planning Cost

# Data - Coupled
coupled_planning_runtime = data_2.iloc[:, -2]   # Coupled Planning Runtime
coupled_planning_cost = data_2.iloc[:, -1]  # Coupled Planning Cost

# Plot  - Runtime vs. Number of Agents
plt.plot(agents, prioritized_planning_runtime, 'b', marker='D', mfc='lightsteelblue', label='Prioritized Planning')
plt.plot(agents, coupled_planning_runtime, 'r', marker='D', mfc='orange', label='Coupled Planning')
#plt.axis([0, 32, 0, 0.01])  # [xmin, xmax, ymin, ymax] 
plt.xlabel('Number of Agents')  
plt.ylabel('Runtime (s)')
plt.title('Runtime vs. Number of Agents')
plt.legend(['Prioritized Planning', 'Coupled Planning'])
plt.show()

# Plot  - Cost vs. Number of Agents
plt.plot(agents, prioritized_planning_cost, 'b', marker='D', mfc='lightsteelblue', label='Prioritized Planning')
plt.plot(agents, coupled_planning_cost, 'r', marker='D', mfc='orange', label='Coupled Planning')
#plt.axis([0, 32, 0, 0.01])  # [xmin, xmax, ymin, ymax]
plt.xlabel('Number of Agents')
plt.ylabel('Total Cost')
plt.title('Total Cost vs. Number of Agents')
plt.legend(['Prioritized Planning', 'Coupled Planning'])
plt.show()


