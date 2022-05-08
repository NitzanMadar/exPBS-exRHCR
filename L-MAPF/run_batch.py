import subprocess

benchmark = 'WAREHOUSE'  # 'WAREHOUSE'

num_agents = [220, 200, 180, 160, 140, 120, 100]  # warehouse
# num_agents = [, 450, 400, 350, 300, 250, 200, 150, 100]  # sorting

deltas = [0, 1, 2]

# ells = [2, 5, 10, 20, 50, 1000]
ells = [10]

hs = [5]  # : warehouse-
# deltas_per_h = {3: [0, 3, 4, 5, 6, 7], 10: [1, 2, 3]}
# hs = [2, 3]  # : warehouse-
# deltas_per_h = {2: [2, 3, 4, 5, 6], 4: [0, 1, 2, 3, 4]}
priority = 2  # total priority = 3, PBS regular = 2
if priority == 3:
    deltas = [0]
if priority not in [2, 3]:
    ValueError('priority should be 2 (regular PBS) or 3 (total priority)')

# queries = [9]
queries = range(50)

for a in num_agents:
    print(f'%%%%%%%%%%%%%%%%\n num_agents={a}\n%%%%%%%%%%%%%%%%')
    for i in queries:  # create new 36
        for h in hs:
            # deltas = deltas_per_h[h]
            for d in deltas:
                for l in ells:
                    create_and_save = 1 if d == 0 else 0
                    # create_and_save = 0

                    print('-----------------------------------------------------------------------------------------\n'
                          '-----------------------------------------------------------------------------------------')
                    if benchmark == 'WAREHOUSE':
                        run_str = f'python3 ./run_windowed_L-MAPF_test_kiva_warehouse.py -a {a} ' \
                                  f'-c {create_and_save} -d {d} -t {i} -l {l} -r {h} -p {priority}'
                    elif benchmark == 'SORTING':
                        run_str = f'python3 ./run_windowed_L-MAPF_test_sorting_center.py -a {a} ' \
                                  f'-c {create_and_save} -d {d} -t {i} -l {l} -r {h} -p {priority}'
                    else:
                        ValueError('illegal benchmark name')
                    print(run_str)
                    print('-----------------------------------------------------------------------------------------\n'
                          '-----------------------------------------------------------------------------------------\n')
                    # input(f'run?')
                    status = subprocess.call(run_str, shell=True)  # if d>0 else 0

                    # run also total priority
                    if d == 0:
                        if benchmark == 'WAREHOUSE':
                            run_str_tot = f'python3 ./run_windowed_L-MAPF_test_kiva_warehouse.py -a {a} ' \
                                          f'-c 0 -d 0 -t {i} -l {l} -r {h} -p 3'
                        elif benchmark == 'SORTING':
                            run_str_tot = f'python3 ./run_windowed_L-MAPF_test_sorting_center.py -a {a} ' \
                                          f'-c 0 -d 0 -t {i} -l {l} -r {h} -p 3'
                        else:
                            ValueError('illegal benchmark name')

                        print(
                            "**************************************\n" + run_str_tot + "\n**************************************\n")
                        status = subprocess.call(run_str_tot, shell=True)
