# Grid-Localization-With-Bayes-Filter
The objective of this project was to find the location of the robot using bayes filter to find the spot with maximum probability of where the robot can be using some predefined object tags in the robotic space.

# Grid Localization
Grid Localization is a variant of discrete Bayes Localization. In this method, the map
is an occupancy grid. At each time step, the algorithm finds out the probabilities of
the robot presence at every grid cell. The grid cells with maximum probabilities at each
step, characterize the robot’s trajectory. Grid Localization runs in two iterative steps —
Movement and Observation.
After each movement, you should compute if that movement can move the robot between
grid cells. For each observation, you should find the most probable cells where that
measurement could have occurred.
