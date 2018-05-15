# Inverted-Pendulum-Deep-Reinforcement-Learning-PSO.
Swarm Particle Optimization to balance a cart pole.

Readme:
1. Our main code file is 'projectmain.py'. Change the lines 78,80,81 to change the parameters as shown in the report for necessary conditions. This is unavoidable to get the best results.
2. Please replace your file with our 'cartpole.py' in gym -> envs -> classic_control -> to avoid any troubles. 
    In the same file, run as it is for noise free condition
                      uncomment lines 31, 32 and 33 for uniform actuator noise 5%
                      uncomment lines 31, 34 and 35 for uniform actuator noise 10%
                      uncomment lines 65, 66 for uniform sensor noise 5%  
                      uncomment lines 65, 67 for uniform sensor noise 10%
                      uncomment lines 65, 68 for Gaussian sensor noise var= 0.1
                      uncomment lines 65, 69 for Gaussian sensor noise var= 0.2
3. Please also replace your file with '__init__.py' in gym -> envs -> to run the code for 60000 time steps. 

Note: The parameters generated in our code are genrated randomly for several environements. Putting a seed and running them would be a real problem because we have to do this on several runs and for several trials. So the results may not be exactly correct but they will be close enough. 
