from subprocess import run

def create_config_file(prob, error, loss, speed):
    """Create a temporary config file with the specified parameters"""
    config_content = f"""[Simulation]
iterations = 10000
dt = 1E-3

[Error]
error = {error}

[Rates]
num_rates = 20
rates = 2000.000, 1000.000, 500.000, 333.333, 250.000, 200.000, 166.667, 142.857, 125.000, 111.111, 100.000, 90.909, 83.333, 76.923, 71.429, 66.667, 62.500, 58.824, 55.556, 52.632, 50.000

[Threads]
num_threads = 14

[Speed]
speed = {speed}

[Drones]
num_drones = 2

[System]
prob = {prob}
loss = {loss * 1000.0}

[Communication]
Ptx = 0.5
Prx = 0.5
Pint = 0.0

[Help]
; This config file contains all parameters for the Vocabs2 drone simulator
; Parameters can be overridden via command line arguments
"""
    
    # Write to a temporary config file
    with open("temp_config.ini", "w") as f:
        f.write(config_content)
    
    return "temp_config.ini"

for s in range(5, 65):
    for loss in [i/10 for i in range(1, 11)]:
        config_file = create_config_file("A", 0.0, loss, s)
        run(["./build/vocabs2", "-c", config_file])
