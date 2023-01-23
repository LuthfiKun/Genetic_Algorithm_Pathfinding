from shapely.geometry import Polygon
from shapely.geometry import Point
from shapely.geometry import LineString
import numpy as np
import numpy.random as rand

class PlayArea:

    def __init__(self, len_X, len_Y):
        #Initiate search state
        self.len_X = len_X
        self.len_Y = len_Y
        self.obstacle = []

    def generate_obstacle(self, num_of_obstacle = 5):
        #Generate obstacle
        self.obstacle = []
        for i in range(num_of_obstacle):
            #Generate random number [0.5, 1] for length from mid point to 4 corners
            point_deviation = rand.randint(1, 2)
            #Generate mid point of the obstacle
            mid_point = rand.randint(0, self.len_X), rand.randint(0, self.len_Y)
            #Generate 4 corner of the obstacle
            top_left = Point(mid_point[0]-point_deviation, mid_point[1]-point_deviation)
            top_right = Point(mid_point[0]+point_deviation, mid_point[1]-point_deviation)
            bottom_left = Point(mid_point[0]-point_deviation, mid_point[1]+point_deviation)
            bottom_right = Point(mid_point[0]+point_deviation, mid_point[1]+point_deviation)
            #Generate obstacle with the 4 corner
            obs = Polygon([top_left, top_right, bottom_right, bottom_left])
            self.obstacle.append(obs)

    def check_collision(self, start, end):
        #Collision is check by calculating intersection area between obstacle and line made from chromosome
        collision = 0 #Variable to save intersection area
        for obs in self.obstacle:
            #Make line from start point to end point (one chromosome to other)
            s_point = Point(start[0], start[1])
            e_point = Point(end[0], end[1])
            line = LineString([s_point, e_point])
            #Check collision for each obstacle
            if obs.intersection(line):
                collision += 1

        return collision

    def check_nearmiss(self, start, end):
        #Collision is check by calculating intersection area between obstacle and line made from chromosome
        collision = 0 #Variable to save intersection area
        for obs in self.obstacle:
            #Make line from start point to end point (one chromosome to other)
            s_point = Point(start[0], start[1])
            e_point = Point(end[0], end[1])
            line = LineString([s_point, e_point])
            #Check collision for each obstacle
            collision += (obs.exterior.distance(line)+1)**(-1)

        return collision

class Robot:

    def __init__(self, start_point, fire_point, pArea, chrom_len, init_chrom = []):
        self.chrom_len = chrom_len #Chromosome length
        self.end_goal = len(fire_point) #Total fire point
        self.start_point = start_point #Starting point
        self.end_point = self.shuffle_firepoint(fire_point) #Shuffle fire point
        self.pArea = pArea
        if len(init_chrom) == 0: #If generationg new individual
            self.chromosome = self.generate_chromosome(pArea) #Generate chromosome
        else: #If generating child
            self.chromosome = init_chrom #Chromosome taken from crossover
        self.calc_fitness()

    def set_chromosome(self, new_chrom):
        self.chromosome = new_chrom
        self.calc_fitness()

    def calc_fitness(self):
        #Calculate fitness
        self.fitness = self.fit_coll(self.pArea)*60 + self.fit_distance()*3 + self.fit_len()*15 + self.fit_angle()*8

    def generate_chromosome(self, pArea):
        chromosome = []
        for i in range(self.end_goal): #Generate chromosome for every fire point
            chrom = []
            for ii in range(self.chrom_len):
                x = rand.randint(0,pArea.len_X) #Random value between 0 and maximum X in play area
                y = rand.randint(0,pArea.len_Y) #Random value between 0 and maximum Y in play area
                chrom.append([x, y])
            chromosome.append(chrom)
        
        return chromosome

    def shuffle_firepoint(self, fire_point):
        #Shuffle fire point
        end_point = fire_point
        rand.shuffle(end_point)
        return end_point

    def fit_len(self):
        #Check fitness by path length
        fitness = 0
        for i in range(len(self.chromosome)):
            for ii in range(self.chrom_len+1):
                if i == 0 and ii == 0:
                    #If first chromosome AND first gen, start line from starting point and end on first gen
                    s_point = Point(self.start_point[0], self.start_point[1])
                    e_point = Point(self.chromosome[i][ii][0], self.chromosome[i][ii][1])
                elif i != 0 and ii == 0:
                    #If NOT first chromosome AND first gen, start line from fire point and end on first gen
                    s_point = Point(self.end_point[i-1][0], self.end_point[i-1][1])
                    e_point = Point(self.chromosome[i][ii][0], self.chromosome[i][ii][1])
                elif ii == self.chrom_len:
                    #If last gen then start line from last gen and end on next fire point
                    s_point = Point(self.chromosome[i][ii-1][0], self.chromosome[i][ii-1][1])
                    e_point = Point(self.end_point[i][0], self.end_point[i][1])
                else:
                    #If not all above then start from i chromosome then end on i+1 chromosome
                    s_point = Point(self.chromosome[i][ii-1][0], self.chromosome[i][ii-1][1])
                    e_point = Point(self.chromosome[i][ii][0], self.chromosome[i][ii][1])
                line = LineString([s_point, e_point]) #Make line between two point
                fitness += line.length #Add length to fitness

        return fitness
    
    def fit_coll(self, playArea):
        #Check fitness by intersection area with obstacle
        fitness = 1
        for i in range(self.end_goal):
            for ii in range(self.chrom_len+1):
                #Logic for making line is the same as fit_len()
                if i == 0 and ii == 0:
                    s_point = self.start_point
                    e_point = self.chromosome[i][ii]
                elif i != 0 and ii == 0:
                    s_point = self.end_point[i-1]
                    e_point = self.chromosome[i][ii]
                elif ii == self.chrom_len:
                    s_point = self.chromosome[i][ii-1]
                    e_point = self.end_point[i]
                else:
                    s_point = self.chromosome[i][ii-1] 
                    e_point = self.chromosome[i][ii]
                collision = playArea.check_collision(s_point, e_point) #Check collision
                fitness += collision #Add collision area to fitness

        return fitness**2

    def fit_angle(self):
        fitness = 0
        for i in range(self.end_goal):
            for ii in range(self.chrom_len):
                if i == 0 and ii == 0:
                    #If first chromosome AND first gen, start line from starting point and end on first gen
                    s_point = Point(self.start_point[0], self.start_point[1])
                    m_point = Point(self.chromosome[i][ii][0], self.chromosome[i][ii][1])
                    e_point = Point(self.chromosome[i][ii+1][0], self.chromosome[i][ii+1][1])
                elif i != 0 and ii == 0:
                    #If NOT first chromosome AND first gen, start line from fire point and end on first gen
                    s_point = Point(self.end_point[i-1][0], self.end_point[i-1][1])
                    m_point = Point(self.chromosome[i][ii][0], self.chromosome[i][ii][1])
                    e_point = Point(self.chromosome[i][ii+1][0], self.chromosome[i][ii+1][1])
                elif ii == self.chrom_len-1:
                    #If last gen then start line from last gen and end on next fire point
                    s_point = Point(self.chromosome[i][ii-1][0], self.chromosome[i][ii-1][1])
                    m_point = Point(self.chromosome[i][ii][0], self.chromosome[i][ii][1])
                    e_point = Point(self.end_point[i][0], self.end_point[i][1])
                else:
                    #If not all above then start from i chromosome then end on i+1 chromosome
                    s_point = Point(self.chromosome[i][ii-1][0], self.chromosome[i][ii-1][1])
                    m_point = Point(self.chromosome[i][ii][0], self.chromosome[i][ii][1])
                    e_point = Point(self.chromosome[i][ii+1][0], self.chromosome[i][ii+1][1])
                line1 = LineString([s_point, m_point]).length + 1 #Make line between two point
                line2 = LineString([m_point, e_point]).length + 1
                cos = 0
                if line1 < line2:
                    cos = (line1/line2) + 1
                else:
                    cos = (line2/line1) + 1
                fitness += cos #Add length to fitness

        return fitness

    def fit_distance(self):
        fitness = 0
        for i in range(self.end_goal-1):
            if i == 0:
                s_point = Point(self.start_point[0], self.start_point[1])
                e_point = Point(self.end_point[i][0], self.end_point[i][1])
            else:
                e_point = Point(self.end_point[i][0], self.end_point[i][1])
                e_point = Point(self.end_point[i+1][0], self.end_point[i+1][1])
            line = LineString([s_point, e_point])
            for ii in range(self.chrom_len):
                point = Point(self.chromosome[i][ii][0], self.chromosome[i][ii][0])
                fitness += line.distance(point)

        return fitness

    def fit_nearmiss(self, playArea):
        fitness = 0
        for i in range(self.end_goal):
            for ii in range(self.chrom_len+1):
                #Logic for making line is the same as fit_len()
                if i == 0 and ii == 0:
                    s_point = self.start_point
                    e_point = self.chromosome[i][ii]
                elif i != 0 and ii == 0:
                    s_point = self.end_point[i-1]
                    e_point = self.chromosome[i][ii]
                elif ii == self.chrom_len:
                    s_point = self.chromosome[i][ii-1]
                    e_point = self.end_point[i]
                else:
                    s_point = self.chromosome[i][ii-1] 
                    e_point = self.chromosome[i][ii]
                collision = playArea.check_nearmiss(s_point, e_point) #Check collision
                fitness += collision #Add collision area to fitness

        return fitness
            

class Genetic_Algoritm:
    def __init__(self, start_point, fire_point, pArea, chrom_len = 10, max_population = 100, 
                    max_iteration = 10, mating_pool = 80, tour_length = 4, selection_param = 0.8,
                    mutation_rate = 0.5, mutate_point = 3, mutation_step = 4):
        self.max_population = max_population #Max Population
        self.chrom_len = chrom_len #Chromosome length
        self.start_point = start_point #Starting point
        self.end_goal = len(fire_point) #Total fire point
        self.population = self.generate_population(start_point, fire_point, pArea, chrom_len) #Generate new population
        self.max_iter = max_iteration #Max iteration
        self.mat_pool = mating_pool #Mating pool
        self.pArea = pArea #Area where robot was deployed
        self.tour_length = tour_length #How many participant each tournament
        self.select_param = selection_param #Alpha for arithmetic crossover
        self.mutate_point = mutate_point #How many gen to mutate
        self.mutation_rate = mutation_rate #Mutation probability
        self.mutation_step = mutation_step #How far from current value a gen will mutate

    def generate_population(self, start_point, fire_point, pArea, chrom_len):
        #Make robot by max population
        population = []
        for i in range(self.max_population):
            robot = Robot(start_point, fire_point, pArea, chrom_len)
            population.append(robot)

        return population

    def selection_parent(self):
        mating_pool = []
        for i in range(self.mat_pool):
            tour_pool = []
            #Generate tour pool, tour participant by parameter 'tour_length', selection by random
            for ii in range(self.tour_length+1): 
                tour_pool.append(self.population[rand.randint(0, self.max_population-1)])
            best_fit = 100000000000
            best_indv = 0
            #Finding best fit in each tournament
            for ii in range(len(tour_pool)):
                if tour_pool[ii].fitness < best_fit:
                    best_fit = tour_pool[ii].fitness
                    best_indv = ii
            mating_pool.append(tour_pool[best_indv]) #Add fittess to mating pool

        return mating_pool

    def crossOver(self):
        mating_pool = self.selection_parent()
        child_pop = []
        for i in range(0,len(mating_pool),2):
            #Choose parent from mating pool
            parent_1 = mating_pool[i]
            parent_2 = mating_pool[i+1]
            child_chrom_1 = []
            child_chrom_2 = []
            for ii in range(self.end_goal):
                cross_point = rand.randint(0, self.chrom_len-1, 2) #Generate two cross point
                #If later bigger than first, swap the two
                if cross_point[0] > cross_point[1]:
                    temp = cross_point[0]
                    cross_point[0] = cross_point[1]
                    cross_point[1] = temp
                #Make the split between two point
                split_1 = parent_2.chromosome[ii][cross_point[0]:cross_point[1]]
                split_2 = parent_1.chromosome[ii][cross_point[0]:cross_point[1]]
                #Combine the split with cromosome from parent
                child_chrom_1.append(parent_1.chromosome[ii][:cross_point[0]]+split_1+parent_1.chromosome[ii][cross_point[1]:])
                child_chrom_2.append(parent_2.chromosome[ii][:cross_point[0]]+split_2+parent_2.chromosome[ii][cross_point[1]:])
            #Make two new individual
            child_chrom_1 = self.mutation(child_chrom_1)
            child_chrom_2 = self.mutation(child_chrom_2)
            child_1 = Robot(self.start_point, parent_1.end_point, self.pArea, self.chrom_len, child_chrom_1)
            child_2 = Robot(self.start_point, parent_2.end_point, self.pArea, self.chrom_len, child_chrom_2)
            #Add to child list
            child_pop.append(child_1)
            child_pop.append(child_2)

        return child_pop

    def mutation(self, chrom_to_mutate):
        mutate =  rand.uniform(0,1)
        for i in range(self.end_goal):
            if mutate <= self.mutation_rate:
                mut_point = rand.randint(0, self.chrom_len-1, self.mutate_point)
                for point in mut_point:
                    new_gen_x = chrom_to_mutate[i][point][0] + rand.randint(-self.mutation_step, self.mutation_step)
                    new_gen_y = chrom_to_mutate[i][point][1] + rand.randint(-self.mutation_step, self.mutation_step)
                    chrom_to_mutate[i][point] = [new_gen_x, new_gen_y]
        
        return chrom_to_mutate


    def elitism(self, top = 0):
        elite = []
        if top == 0: #Used when finding top 1
            top = self.max_population - self.mat_pool
        pool = self.population.copy()
        for i in range(top):
            best_fit = 100000000000
            best_indv = -1
            for ii in range(len(pool)):
                #Find the best individual in the pool
                if pool[ii].fitness < best_fit:
                    best_fit = pool[ii].fitness
                    best_indv = ii

            elite.append(pool[best_indv]) #Add the best individual to elite pool
            pool.pop(best_indv) #Pop from pool, then continue to find second best 

        return elite

    def run(self):
        for i in range(self.max_iter):

            child_pool = self.crossOver()
            elite_pool = self.elitism()
            self.population = elite_pool + child_pool

            print('Best Fitness Generation {0} : {1}'.format(i, self.elitism(1)[0].fitness))

            #fitness_hist.append(self.elitism(1)[0].fitness)
        
        best_indv = self.elitism(1)

        return best_indv[0]