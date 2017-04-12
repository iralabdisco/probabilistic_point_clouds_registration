#!/usr/bin/env python
# -*- coding: utf-8 -*-

import operator
import random
import subprocess

import numpy

from deap import base
from deap import benchmarks
from deap import creator
from deap import tools

source_cloud = "bun045.pcd"
target_cloud = "bun000.pcd"

creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
creator.create("Particle", list, fitness=creator.FitnessMin, speed=list,
    smin=None, smax=None, best=None)

def generate(pmin, pmax, smin, smax):
    #particle state is [tx, ty, tz, roll, pitch, yaw, radius]

    state = []
    for min, max in zip(pmin, pmax):
        state.append(random.uniform(min,max))
    part = creator.Particle(state)

    speed = []
    for min, max in zip(smin, smax):
        speed.append(random.uniform(min,max))
    part.speed = speed
    part.smin = smin
    part.smax = smax
    return part

def updateParticle(part, best, phi1, phi2):
    u1 = (random.uniform(0, phi1) for _ in range(len(part)))
    u2 = (random.uniform(0, phi2) for _ in range(len(part)))
    v_u1 = map(operator.mul, u1, map(operator.sub, part.best, part))
    v_u2 = map(operator.mul, u2, map(operator.sub, best, part))
    part.speed = list(map(operator.add, part.speed, map(operator.add, v_u1, v_u2)))
    for i, speed in enumerate(part.speed):
        if speed < part.smin[i]:
            part.speed[i] = part.smin[i]
        elif speed > part.smax[i]:
            part.speed[i] = part.smax[i]
    part[:] = list(map(operator.add, part, part.speed))

def evaluate(individual):
    tx = individual[0]
    ty = individual[1]
    tz = individual[2]

    roll = individual[3]
    pitch = individual[4]
    yaw = individual[5]

    radius = individual[6]

    score = subprocess.check_output(["../../../build-Release/point_cloud_registration","bun090_UnStructured.pcd", "bun000_UnStructured.pcd", "-m 0" ,
    "-s 0.01", "-t 0.01",
    "-i 1","-d 5", "-r "+str(radius), "--tx "+str(tx), "--ty "+str(ty), "--tz "+str(tz), "--roll "+str(roll), "--pitch "+str(pitch), "--yaw "+str(yaw)])
    return float(score),
#intertial 0.72, cognitive 1.49, social 1.49
toolbox = base.Toolbox()
pmin = [-1,-1,-1,0,0,0,0.01]
pmax = [1,1,1,360,360,360,1]
smin = list(pmin)
smax = list(pmax)
toolbox.register("particle", generate, pmin, pmax, smin, smax)
toolbox.register("population", tools.initRepeat, list, toolbox.particle)
toolbox.register("update", updateParticle, phi1=2.0, phi2=2.0)
toolbox.register("evaluate", evaluate)

def main():
    pop = toolbox.population(n=50)
    stats = tools.Statistics(lambda ind: ind.fitness.values)
    stats.register("avg", numpy.mean)
    stats.register("std", numpy.std)
    stats.register("min", numpy.min)
    stats.register("max", numpy.max)

    logbook = tools.Logbook()
    logbook.header = ["gen", "evals"] + stats.fields

    GEN = 1000
    best = None

    for g in range(GEN):
        for part in pop:
            part.fitness.values = toolbox.evaluate(part)
            if not part.best or part.best.fitness < part.fitness:
                part.best = creator.Particle(part)
                part.best.fitness.values = part.fitness.values
            if not best or best.fitness < part.fitness:
                best = creator.Particle(part)
                best.fitness.values = part.fitness.values
        for part in pop:
            toolbox.update(part, best)

        # Gather all the fitnesses in one list and print the stats
        logbook.record(gen=g, evals=len(pop), **stats.compile(pop))
        print(logbook.stream)
        print "Best: " + str(best)
    return pop, logbook, best

if __name__ == "__main__":
    main()
