import numpy as np
import retro
import pygame
import cv2
import imutils
import random
# import rospy
#from geometry_msgs.msg import Twist

# Initial population
# Every single individuous is a solution to the system, which means
# that have enough step to complete the level at least 8000 steps
# every step have the following combination
# ["B", "A", "MODE", "START", "UP", "DOWN", "LEFT", "RIGHT", "C", "Y", "X", "Z"]
# The initial population have 100 individuous
# population = np.random.randint(2, size=(100,8000,12))

#
# Global variables
# Setup optimal string and GA input variables.
#

OPTIMAL     = 7000
DNA_SIZE    = 8000
POP_SIZE    = 100
GENERATIONS = 5000
video_size = 700, 500

#
# Helper functions
# These are used as support, but aren't direct GA-specific functions.
#

def weighted_choice(items):
  """
  Chooses a random element from items, where items is a list of tuples in
  the form (item, weight). weight determines the probability of choosing its
  respective item. Note: this function is borrowed from ActiveState Recipes.
  """
  weight_total = sum((item[1] for item in items))
  n = random.uniform(0, weight_total)
  for item, weight in items:
    if n < weight:
      return item
    n = n - weight
  return item

def random_step():
  return np.random.randint(2, size=12)

def random_population():
  return np.random.randint(2, size=(POP_SIZE, DNA_SIZE,12))

#
# GA functions
# These make up the bulk of the actual GA algorithm.
#

def fitness(individual_advance):
  fitness_val = OPTIMAL - individual_advance
  return fitness_val

def mutate(individual):
  """
  For each gene in the DNA, there is a 1/mutation_chance chance that it will be
  switched out with a random character. This ensures diversity in the
  population, and ensures that is difficult to get stuck in local minima.
  """
  mutation_chance = 100
  dna_out = individual
  for c in range(DNA_SIZE):
      if int(random.random()*mutation_chance) == 1:
          dna_out[c] = random_step()
  return dna_out

def crossover(dna1, dna2):
  """
  Slices both dna1 and dna2 into two parts at a random index within their
  length and merges them. Both keep their initial sublist up to the crossover
  index, but their ends are swapped.
  """
  pos = int(random.random()*DNA_SIZE)
  return (np.concatenate((dna1[:pos],dna2[pos:]), axis=0), np.concatenate((dna2[:pos],dna1[pos:]), axis=0))

#
# Main driver
# Generate a population and simulate GENERATIONS generations.
#

if __name__ == "__main__":
    # Generate initial population. This will create a list of POP_SIZE strings,
    # each initialized to a sequence of random characters.
    population = random_population()
    # Simulate all of the generations.
    generation=1
    env = retro.make(game='SonicTheHedgehog-Genesis', state='GreenHillZone.Act1')
    screen = pygame.display.set_mode(video_size)
    for _ in range(GENERATIONS):
        for individual in population:
            weighted_population = []
            observation = env.reset()
            info = None
            for step in individual:
                img = env.render(mode='rgb_array')
                # ROTATE THE IMAGE THE MATRIX IS 90 grates and mirror
                cv2.putText(img, "Gen: {} - {}".format(generation, step), (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                _observation, _reward, _done, info = env.step(step)
                cv2.putText(img, "Fitness: {}".format(fitness(info['x'])), (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                img = np.flipud(np.rot90(img))
                image_np = imutils.resize(img, width=500)
                surf = pygame.surfarray.make_surface(image_np)
                screen.blit(surf, (0, 0))
                pygame.display.update()
            # Add individuals and their respective fitness levels to the weighted
            # population list. This will be used to pull out individuals via certain
            # probabilities during the selection phase. Then, reset the population list
            # so we can repopulate it after selection.
            fitness_val = fitness(info['x'])
            # Generate the (individual,fitness) pair, taking in account whether or
            # not we will accidently divide by zero.
            if fitness_val == 0:
                pair = (individual, 1.0)
            else:
                pair = (individual, 1.0/fitness_val)
            weighted_population.append(pair)
        # Select two random individuals, based on their fitness probabilites, cross
        # their genes over at a random point, mutate them, and add them back to the
        # population for the next iteration.
        population = []
        for _ in range(int(POP_SIZE/2)):
            # Selection
            ind1 = weighted_choice(weighted_population)
            ind2 = weighted_choice(weighted_population)
            # Crossover
            ind1, ind2 = crossover(ind1, ind2)
            # Mutate and add back into the population.
            population.append(mutate(ind1))
            population.append(mutate(ind2))
        generation+=1
    # Display the highest-ranked string after all generations have been iterated
    # over. This will be the closest string to the OPTIMAL string, meaning it
    # will have the smallest fitness value. Finally, exit the program.
    fittest_string = population[0]
    for step in population[0]:
        img = env.render(mode='rgb_array')
        # ROTATE THE IMAGE THE MATRIX IS 90 grates and mirror
        cv2.putText(img, "EVALUATE: {} ".format(step), (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        _observation, _reward, _done, info = env.step(step)
        cv2.putText(img, "Fitness: {}".format(fitness(info['x'])), (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        img = np.flipud(np.rot90(img))
        image_np = imutils.resize(img, width=500)
        surf = pygame.surfarray.make_surface(image_np)
        screen.blit(surf, (0, 0))
        pygame.display.update()
    minimum_fitness = fitness(info['x'])
    for individual in population:
        img = env.render(mode='rgb_array')
        # ROTATE THE IMAGE THE MATRIX IS 90 grates and mirror
        cv2.putText(img, "EVALUATE: {} ".format(step), (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        _observation, _reward, _done, info = env.step(step)
        cv2.putText(img, "Fitness: {}".format(fitness(info['x'])), (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        img = np.flipud(np.rot90(img))
        image_np = imutils.resize(img, width=500)
        surf = pygame.surfarray.make_surface(image_np)
        screen.blit(surf, (0, 0))
        pygame.display.update()
        ind_fitness = fitness(info['x'])
        if ind_fitness <= minimum_fitness:
            fittest_string = individual
            minimum_fitness = ind_fitness
            print ("Fittest String: %s" % fittest_string)
    exit(0)
