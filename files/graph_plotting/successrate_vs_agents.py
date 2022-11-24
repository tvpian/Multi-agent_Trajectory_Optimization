# Plot Runtime (s) vs. Number of Agents
# for the 2 different algorithms

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# Read in data


# Data - 
agents = [4, 8, 16, 24, 32]    # Number of agents
random_strategy_rate = [100, 100, 100, 85, 75]   # Random Strategy success rate
proposed_strategy_rate = [100, 100, 100, 100, 85]  # Proposed Strategy success rate

# Plot  - Success_rate vs. Number of Agents
plt.plot(agents, random_strategy_rate, 'b', marker='D', mfc='lightsteelblue', label='Random Strategy')
plt.plot(agents, proposed_strategy_rate, 'r', marker='D', mfc='orange', label='Proposed Strategy')
plt.axis([0, 34, 0, 105])  # [xmin, xmax, ymin, ymax]
plt.xlabel('Number of Agents')
plt.ylabel('Success Rate (%)')
plt.title('Success Rate vs. Number of Agents')
plt.legend(['Random Strategy', 'Proposed Strategy'])
plt.show()

