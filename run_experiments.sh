# python3.11 main.py --dataset=AC300 --route_file_name=mem_inf_route_data0 --experiment_name=ours_4robots_600capacity_AC300 --n_robots=4 --capacity=600 > ours_4robots_600capacity_AC300.txt &
# python3.11 main.py --dataset=AC300 --route_file_name=mem_inf_route_data0 --experiment_name=ours_naive_4robots_600capacity_AC300 --n_robots=4 --capacity=600 --point_estimation=True > ours_naive_4robots_600capacity_AC300.txt &
# python3.11 main.py --dataset=AC300 --route_file_name=mem_inf_route_data0 --experiment_name=ours_2robots_1200capacity_AC300 --n_robots=2 --capacity=1200 > ours_2robots_1200capacity_AC300.txt &
# python3.11 main.py --dataset=AC300 --route_file_name=mem_inf_route_data0 --experiment_name=ours_naive_2robots_1200capacity_AC300 --n_robots=2 --capacity=1200 --point_estimation=True > ours_naive_2robots_1200capacity_AC300.txt &
# python3.11 main.py --dataset=AC300 --route_file_name=mem_inf_route_data0 --experiment_name=ours_1robots_infcapacity_AC300 --n_robots=1 > ours_1robots_infcapacity_AC300.txt & 
# python3.11 main.py --dataset=AC300 --route_file_name=mem_inf_route_data0 --experiment_name=ours_naive_1robots_infcapacity_AC300 --n_robots=1 --point_estimation=True > ours_naive_1robots_infcapacity_AC300.txt & 

# python3 main.py --dataset=VM25 --route_file_name=mem_r1_route_data0 --experiment_name=ours_1robots_1600capacity_repeat30_VM25_13 --n_robots=1 --capacity=2000  > ours_1robots_1600capacity_repeat30_VM25_13.txt &
# python3 main.py --dataset=VM25 --route_file_name=mem_r1_route_data0 --experiment_name=ours_2robots_1200capacity_repeat30_VM25_13 --n_robots=2 --capacity=1200  > ours_2robots_1200capacity_repeat30_VM25_13.txt &
# python3 main.py --dataset=VM25 --route_file_name=mem_r1_route_data0 --experiment_name=ours_3robots_800capacity_repeat30_VM25_13 --n_robots=3 --capacity=800  > ours_3robots_800capacity_repeat30_VM25_13.txt &
# python3 main.py --dataset=VM25 --route_file_name=mem_r1_route_data0 --experiment_name=ours_4robots_480capacity_repeat30_VM25_13 --n_robots=4 --capacity=480 > ours_4robots_480capacity_repeat30_VM25_13.txt &
# python3 main.py --dataset=VM25 --route_file_name=mem_r1_route_data0 --experiment_name=ours_5robots_400capacity_repeat30_VM25_13 --n_robots=5 --capacity=400 > ours_5robots_400capacity_repeat30_VM25_13.txt &
# python3 main.py --dataset=VM25 --route_file_name=mem_r1_route_data0 --experiment_name=ours_8robots_400capacity_repeat30_VM25_13 --n_robots=8 --capacity=350 > ours_8robots_300capacity_repeat30_VM25_13.txt &


python3.11 main.py --dataset=AC300 --route_file_name=mem_inf_route_data0 --experiment_name=point_14robots_200capacity_AC300 --point_estimation=True --n_robots=14 --capacity=200 > naive_14robots_200capacity_AC300.tx &
python3.11 main.py --dataset=AC300 --route_file_name=mem_inf_route_data0 --experiment_name=point_12robots_200capacity_AC300 --point_estimation=True --n_robots=12 --capacity=200 > naive_12robots_200capacity_AC300.tx &
python3.11 main.py --dataset=AC300 --route_file_name=mem_inf_route_data0 --experiment_name=point_10robots_200capacity_AC300 --point_estimation=True --n_robots=10 --capacity=200 > naive_10robots_200capacity_AC300.tx &
python3.11 main.py --dataset=AC300 --route_file_name=mem_inf_route_data0 --experiment_name=point_8robots_300capacity_AC300  --point_estimation=True --n_robots=8 --capacity=300 > naive_8robots_300capacity_AC300.txt &
python3.11 main.py --dataset=AC300 --route_file_name=mem_inf_route_data0 --experiment_name=point_6robots_400capacity_AC300  --point_estimation=True --n_robots=6 --capacity=400 > naive_6robots_400capacity_AC300.txt &
python3.11 main.py --dataset=AC300 --route_file_name=mem_inf_route_data0 --experiment_name=point_4robots_500capacity_AC300  --point_estimation=True --n_robots=4 --capacity=500 > naive_4robots_500capacity_AC300.txt &
python3.11 main.py --dataset=AC300 --route_file_name=mem_inf_route_data0 --experiment_name=point_2robots_1000capacity_AC300 --point_estimation=True --n_robots=2 --capacity=1000 > naive_2robots_1000capacity_AC300.txt &


wait