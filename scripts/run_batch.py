import subprocess

num_agents = [100, 110, 120, 130, 140]
num_agents = [100, 150, 200, 250, 300, 350, 400]

deltas = [0, 1, 2, 3, 4]

# ells = [2, 5, 10, 20, 50, 1000]
ells = [10]
for a in num_agents:
    print(f'%%%%%%%%%%%%%%%%%\n num_agents={a}\n%%%%%%%%%%%%%%%%%')
    for i in range(10):  # create new 36
        for d in deltas:
            for l in ells:

                # create_and_save = 1 if d == 0 else 0
                create_and_save = 0

                print('-----------------------------\n'
                      '-----------------------------\n'
                      '-----------------------------\n'
                      '-----------------------------')

                run_str = f'python3 ./run_windowed_L-MAPF_test_kiva_warehouse.py -a {a} -c {create_and_save} -d {d} -t {i} -l {l}'
                # run_str = f'python3 ./run_windowed_L-MAPF_test_sorting_center.py -a {a} -c {create_and_save} -d {d} -t {i} -l {l}'
                print(run_str)
                print('-----------------------------\n'
                      '-----------------------------\n'
                      '-----------------------------\n'
                      '-----------------------------\n')
                # input(f'run?')
                status = subprocess.call(run_str, shell=True)
