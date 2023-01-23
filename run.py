import genetic_algoritm
import matplotlib.pyplot as plt
import timeit

#Store fitness history
fitness_hist = []

#Generate area
pArea = genetic_algoritm.PlayArea(15,15)
pArea.generate_obstacle(15)

#Generate obstacle
for poly in pArea.obstacle:
    x_val = []
    y_val = []
    x, y = poly.exterior.coords.xy
    plt.fill(x, y, 'ko')

#Start and End point
start = [1,14]
fire = [[8,2]]

#Plot path
plt.plot(start[0],start[1], 'bo')
for i in fire:
    plt.plot(i[0],i[1], 'ro')

plt.show()

#GA Parameter
chrom_len = 3
pop_size = 100
max_generation = 25
iter_num = 5
mating_pool = 80
tour_length = 5
mutation_probability = 0.2
mutate_point = 1
mutation_step = 2
selection_param = 0.3

iter_best = []

start_time = timeit.default_timer()
GA = genetic_algoritm.Genetic_Algoritm(start, fire, pArea, chrom_len, pop_size, max_generation, 
                        mating_pool, tour_length, selection_param, 
                        mutation_probability, mutate_point, mutation_step)
best_indv = GA.run()
stop_time = timeit.default_timer()
print("Iter: ", i)
print("Best Fitness: ", best_indv.fitness)
print("Best Chromosome:", best_indv.chromosome)
print('Runtime: ', stop_time - start_time)

for poly in pArea.obstacle:
    x_val = []
    y_val = []
    x, y = poly.exterior.coords.xy
    plt.fill(x, y, 'ko')

plt.plot(start[0],start[1], 'bo')
for i in fire:
    plt.plot(i[0],i[1], 'ro')

for i in range(GA.end_goal):
    for ii in range(GA.chrom_len+1):
        if i == 0 and ii == 0:
            s_point = best_indv.start_point
            e_point = best_indv.chromosome[i][ii]
        elif i != 0 and ii == 0:
            s_point = best_indv.end_point[i-1]
            e_point = best_indv.chromosome[i][ii]
        elif ii == GA.chrom_len:
            s_point = best_indv.chromosome[i][ii-1]
            e_point = best_indv.end_point[i]
        else:
            s_point = best_indv.chromosome[i][ii-1] 
            e_point = best_indv.chromosome[i][ii]
        x_value = [s_point[0], e_point[0]]
        y_value = [s_point[1], e_point[1]]
        plt.plot(x_value, y_value)

plt.axis([-1, pArea.len_X+1, -1, pArea.len_Y+1])
plt.show()

plt.clf()
plt.plot(range(0, max_generation), GA.history)
plt.show()



