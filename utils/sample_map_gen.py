import numpy as np
import pandas as pd

# Dimensions based on the original data
rows = 157
cols = 136

# ---------------------------------------------------------
# 1. Generate the Elevation Map
# ---------------------------------------------------------
# Create a matrix where all values are 40.0
elevation_matrix = np.full((rows, cols), 40.0)

# Save to CSV using SPACE as the separator
elevation_file = 'generated_elevationMap_40.csv'
pd.DataFrame(elevation_matrix).to_csv(elevation_file, index=False, header=False, sep=' ')


# ---------------------------------------------------------
# 2. Generate the Soil Map
# ---------------------------------------------------------
# Create a matrix where all initial values are 1
soil_matrix = np.full((rows, cols), 1, dtype=int)

# Set all the outer edges to 0
soil_matrix[0, :] = 0      # Top row
soil_matrix[-1, :] = 0     # Bottom row
soil_matrix[:, 0] = 0      # Leftmost column
soil_matrix[:, -1] = 0     # Rightmost column

# Save to CSV using SPACE as the separator
soil_file = 'generated_soilMap_edges_0.csv'
pd.DataFrame(soil_matrix).to_csv(soil_file, index=False, header=False, sep=' ')

print(f"Successfully generated '{elevation_file}' and '{soil_file}'.")