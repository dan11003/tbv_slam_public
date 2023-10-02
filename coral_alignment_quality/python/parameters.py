# Parameters for experiments

# Define a list per each parameter here (add as many as necessary)
rangeError = [0.001]
range error=[0.05, 0.1, 0.15, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9] # [0.3 0.6 0.9]
r=[0.1 0.3 0.5 0.7 0.9 1.1 1.4 1.7 2.0 2.5 3]
methods = ['Coral','P2P','P2D','P2L'] #['Coral'] #['P2P','P2D','P2L']
scanMinDistance = [0, 1, 2, 3, 4, 5, 10, 15, 20, 25] # [0 5 10]
thetaError = [0.0017, 0.003775, 0.00585, 0.007925, 0.01, 0.0184, 0.0268, 0.0352, 0.0436, 0.052]

# Include lists here
parameters = [rangeError, methods, scanMinDistance, thetaError]
