"""

Author: Ben Bausch

"""
import WorldGenerator
import os
from math import *
from collections import defaultdict
import numpy as np


def writeConstans(i, file, num):
  if i == 0:
    file.write('   bs' + str(num) + ' - base_station\n')
    file.write('   bs' + str(num) + '_out - bs_output\n')
    file.write('   bs' + str(num) + '_in - bs_output\n')
  elif i == 1:
    file.write('   rs' + str(num) + ' - ring_station\n')
    file.write('   rs' + str(num) + '_out - rs_output\n')
    file.write('   rs' + str(num) + '_in - rs_input\n')
  elif i == 2:
    file.write('   cs' + str(num) + ' - cap_station\n')
    file.write('   cs' + str(num) + '_out - cs_output\n')
    file.write('   cs' + str(num) + '_in - cs_input\n')
  elif i == 3:
    file.write('   ds' + str(num) + ' - delivery_station\n')
    file.write('   ds' + str(num) + '_in - ds_input\n')


def addActions(i, wfile, rfile):
  """
  This method takes rfile as input and adapts the actions to current robot
  and then writes the lines to the given wfile
  """
  lines = rfile.readlines()
  word1ToChange = "r" + str(i)
  word2ToChange = "robot" + str(i)

  for line in lines:
    t = line.replace("r1", word1ToChange)
    b = t.replace("robot1", word2ToChange)
    wfile.writelines(b)


def addLocations(i, file, num):
  if i == 0:
    file.write('(output-location bs' + str(num) + '_out bs' + str(num) + ')\n   ')
    # TODO deltete : file.write('(input-location bs' + str(num) + '_in bs' + str(num) + ')\n   ')
  elif i == 1:
    file.write('(input-location rs' + str(num) + '_in rs' + str(num) + ')\n   ')
    file.write('(output-location rs' + str(num) + '_out rs' + str(num) + ')\n   ')
  elif i == 2:
    file.write('(input-location cs' + str(num) + '_in cs' + str(num) + ')\n   ')
    file.write('(output-location cs' + str(num) + '_out cs' + str(num) + ')\n   ')
  elif i == 3:
    file.write('(input-location ds' + str(num) + '_in ds' + str(num) + ')\n   ')


def calTime(a, b):
  print(a)
  print('\n')
  print(b)
  print('\n')
  return sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def gS(i):
  """
  :return: returns type of a station
  """
  if i == 1:
    return 'bs'
  elif i == 2:
    return 'rs'
  elif i == 3:
    return 'cs'
  elif i == 4:
    return 'ds'


def intToStr(i):
  if i == 0:
    return 'zero'
  elif i == 1:
    return 'one'
  elif i == 2:
    return 'two'
  elif i == 3:
    return 'three'


def generateDomain(privateVar=True, homogen=True, stations=[1, 2, 2, 1], agentNum=3,
                   products=[["base", "blue", "red", "cap"], ["base", "yellow", "red", "cap"]]):
  """
  This method will generate agentNum Pddl files. These files are the Domains for each robot need for
  the decentralized Planning.

  :param privateVar: should there be private information
                     (useful to limit information flow for faster planning)
  :param homogen: do the agents have the same actions
  :param stations: specific number of different stations
  :param agentNum: number od agents

  :return: no return but generates a PDDL domain file for each agent
  """
  # first we create agentNum different Domain files
  # since
  for i in range(1, agentNum + 1):
    new_file_name = "domain-a" + str(i) + ".pddl"
    f = open(new_file_name, "a")
    f.truncate(0)

    # typing id always the same
    with open('typing.txt', 'r') as typing:
      lines = typing.readlines()
      f.writelines(lines)

    # now we have to define the constants
    f.write('(:constants\n\n')
    # now for the different stations define the type
    for a in range(len(stations)):
      for b in range(0, stations[a]):
        writeConstans(i=a, file=f, num=b)
    f.write('\n   start - s_location \n')
    f.write('\n   zero one two three - material_counter \n)\n\n')

    # now we have to add the knowledge of the robots  about the world, the predicates
    # there is some knowledge the robots always share and some that might not be of interest for other robots
    with open('predicates.txt', 'r') as pred:
      lines = pred.readlines()
      for line in lines:
        b = line.replace('robot1', 'robot' + str(i))
        f.writelines(b)
    if privateVar:
      f.write('\n   (:private \n')
    with open('private.txt', 'r') as priv:
      lines = priv.readlines()
      for line in lines:
        b = line.replace('robot1', 'robot' + str(i))
        f.writelines(b)
    if privateVar:
      # for including private variables we miss a ) at the end
      f.write('\n)\n\n')

    # add functions which are the same for all the robots
    with open('functions.txt', 'r') as func:
      lines = func.readlines()
      f.writelines(lines)

    # now we can add the different actions
    # first the actions which are the same for all the robots
    with open('generalActions.txt', 'r') as genAct:
      lines = genAct.readlines()
      f.writelines(lines)

    if homogen:
      # homogen = they have all the same action at their dispense
      with open('specificActions.txt', 'r') as speAct:
        addActions(i, wfile=f, rfile=speAct)

    f.write('\n)\n')


def generateProblems(privateVar=True, homogen=True, stations=[1, 2, 2, 1], agentNum=3,
                     products=[["silver_base", "blue_ring", "orange_ring", "black_cap"]], stepLocFixed=True,
                     statasign={'robot1': ['bs0', 'ds0'], 'robot2': ['rs0', 'cs0'], 'robot3': ['rs1', 'cs1']}):
  """
  This method will generate agentNum Pddl files. These files are the Problem files for each robot need for
  the decentralized Planning.

  :param stepLocFixed: indicates if the according steps can be done everywhere
  :param privateVar: should there be private information
                     (useful to limit information flow for faster planning)
  :param homogen: do the agents have the same actions
  :param stations: specific number of different stations
  :param agentNum: number od agents
  :param products: is a list of different products, one product is a list of colors with cap and base

  :return: no return bit generates a PDDL domain file for each agent
  """
  # remember the step locations
  stepLoc = defaultdict()

  # assign the machines
  with open('stationAsign.txt', 'w') as pred:
    # define the steps location
    if stepLocFixed:
      counter = 0
      for prod in products:
        for j in range(len(prod)):
          if j == 0:
            ran = np.random.randint(stations[0])
            pred.write('(step-at-machine ' + prod[j] + '_p' + str(counter) + ' bs' + str(ran) + ')\n   ')
            try:
              stepLoc['bs' + str(ran)] += [prod[j] + '_p' + str(counter)]
            except KeyError:
              stepLoc['bs' + str(ran)] = [prod[j] + '_p' + str(counter)]
          elif 'ring' in prod[j]:
            ran = np.random.randint(stations[1])
            pred.write('(step-at-machine ' + prod[j] + '_p' + str(counter) + ' rs' + str(ran) + ')\n   ')
            try:
              stepLoc['rs' + str(ran)] += [prod[j] + '_p' + str(counter)]
            except KeyError:
              stepLoc['rs' + str(ran)] = [prod[j] + '_p' + str(counter)]
          elif 'cap' in prod[j]:
            ran = np.random.randint(stations[2])
            pred.write('(step-at-machine ' + prod[j] + '_p' + str(counter) + ' cs' + str(ran) + ')\n   ')
            try:
              stepLoc['cs' + str(ran)] += [prod[j] + '_p' + str(counter)]
            except KeyError:
              stepLoc['cs' + str(ran)] = [prod[j] + '_p' + str(counter)]
          elif j == len[prod] - 1:
            ran = np.random.randint(stations[3])
            pred.write('(step-at-machine ' + prod[j] + '_p' + str(counter) + ' ds' + str(ran) + ')\n   ')
            try:
              stepLoc['ds' + str(ran)] += [prod[j] + '_p' + str(counter)]
            except KeyError:
              stepLoc['ds' + str(ran)] = [prod[j] + '_p' + str(counter)]
        counter += 1
    else:
      counter = 0
      for prod in products:
        for j in range(len(prod)):
          pass
          # TODO :implement this

  layout = WorldGenerator.generateRadnomLayout(stations[0], stations[1], stations[2], stations[3], agentNum=agentNum)

  for i in range(1, agentNum + 1):
    new_file_name = "problem-a" + str(i) + ".pddl"
    f = open(new_file_name, "a")
    f.truncate(0)

    # object definition
    # the beginning of each file is the same:
    f.write('(define (problem rcll-production-task)\n')
    f.write('(:domain rcll-production-steps)\n')
    f.write('(:objects\n   ')
    # now we define our robots as objects
    for j in range(1, agentNum + 1):
      f.write('r' + str(j) + ' ')
    f.write('- robot\n   ')
    # now the production steps needed
    counter = 0
    for j in products:
      f.write('p' + str(counter) + ' - product\n   ')
      for step in j:
        f.write(step + '_p' + str(counter) + ' ')
      f.write('- step\n   ')
      counter += 1
    f.write(')    \n')
    # for clean format
    size = f.tell()
    f.truncate(size - 3)

    # initialization
    # now we have to initialize our counter predicates
    with open('counterPredicates.txt', 'r') as pred:
      lines = pred.readlines()
      f.writelines(lines)
    # now we can initialize our production predicates
    f.write('   ')
    counter = 0
    for j in products:
      for step in j:
        f.write('(has-step p' + str(counter) + ' ' + step + '_p' + str(counter) + ')\n   ')
      f.write('(initial-step ' + j[0] + '_p' + str(counter) + ')\n   ')
      counter += 1

    # Todo check this:
    with open('stationAsign.txt', 'r') as pred:
      lines = pred.readlines()
      f.writelines(lines)
    for j in range(0, len(stations)):
      for k in range(0, stations[j]):
        addLocations(i=j, file=f, num=k)

    # initialize robot position
    f.write('(robot' + str(i) + '-at-init)\n   ')

    # define robot order
    for j in range(1, agentNum + 1):
      for k in range(j, agentNum + 1):
        if j != k:
          f.write('(robot' + str(j) + '-precedes r' + str(k) + ')\n   ')

    # initaly no material stored in any machine
    for j in range(len(stations)):
      for k in range(0, stations[j]):
        f.write('(material-stored ' + gS(j + 1) + str(k) + ' zero)\n   ')

    # now we have to calculate how much material we need
    # we create a dictionary to store the number needed of a specific material
    matNum = defaultdict()
    counter = 0
    for prod in products:
      for j in range(0, len(prod)):
        try:
          matNum[prod[j]][0] += 1
        except KeyError:
          matNum[prod[j]] = [1, '_p' + str(counter)]
      counter += 1

    # last but not least we have to define the order in which the steps have to be executed
    counter = 0
    for prod in products:
      for j in range(0, len(prod)):
        if j + 1 < len(prod):
          prodcut = '_p' + str(counter)
          f.write('(step-precedes ' + prod[j] + prodcut + ' ' + prod[j + 1] + prodcut + ')\n   ')
      counter += 1

    # now lets add the the material required lines
    for component in matNum.keys():
      f.write(
        '(material-required ' + component + matNum[component][1] + ' ' + intToStr(matNum[component][0]) + ')\n   ')

    # now we have to initialize our world, here the layout is important
    # we calculate the time needed from one position to another one, as the time needed to cross the bee line
    # the transition from one tile to another one equals to 1 time step
    for j in layout:
      for k in layout:
        print('j[2] \n')
        print(j[2])
        print('j[3]\n')
        print(j[3])
        print('k[2]')
        print(k[2])
        print('-------------------------\n')
        if j[2] is None and j[3] is None:
          print('j[0] = ' + str(j[0]))
          if j[0] == i:
            # to input
            if not (k[3] is None):
              t = ceil(calTime(j[1], k[3]))
              print('I got here')
              f.write('(= (path-length ' + 'start ' + gS(k[0]) + str(k[4]) + '_in) ' + str(t) + ')\n   ')
              f.write('(= (path-length ' + gS(k[0]) + str(k[4]) + '_in start) ' + str(t) + ')\n   ')
            # to output
            if not (k[2] is None):
              t = ceil(calTime(j[1], k[2]))
              f.write('(= (path-length ' + 'start ' + gS(k[0]) + str(k[4]) + '_out) ' + str(t) + ')\n   ')
              f.write('(= (path-length ' + gS(k[0]) + str(k[4]) + '_out start) ' + str(t) + ')\n   ')

        elif k[2] is None and k[3] is None:
          pass

        else:
          # input to input
          if not (j[3] is None or k[3] is None):
            t = ceil(calTime(j[3], k[3]))
            f.write(
              '(= (path-length ' + gS(j[0]) + str(j[4]) + '_in ' + gS(k[0]) + str(k[4]) + '_in) ' + str(t) + ')\n   ')
          # input to output
          if not (j[3] is None or k[2] is None):
            t = ceil(calTime(j[3], k[2]))
            f.write(
              '(= (path-length ' + gS(j[0]) + str(j[4]) + '_in ' + gS(k[0]) + str(k[4]) + '_out) ' + str(t) + ')\n   ')
          # output to input
          if not (j[2] is None or k[3] is None):
            t = ceil(calTime(j[2], k[3]))
            f.write(
              '(= (path-length ' + gS(j[0]) + str(j[4]) + '_out ' + gS(k[0]) + str(k[4]) + '_in) ' + str(t) + ')\n   ')
          # output to output
          if not (j[2] is None or k[2] is None):
            t = ceil(calTime(j[2], k[2]))
            f.write(
              '(= (path-length ' + gS(j[0]) + str(j[4]) + '_out ' + gS(k[0]) + str(k[4]) + '_out) ' + str(t) + ')\n   ')

    # we did not cover start to start :
    f.write('(= (path-length start start) 0) \n   ')

    # Assigning the stations to the robots
    if i == 1:
      for val in statasign['robot' + str(i)]:
        f.write('(robot' + str(i) + '-assigned-machine ' + val + ')\n   ')

    # tine already used at the beginning is 0 time steps
    f.write('(= (total-cost) 0)\n   )\n   ')

    # now lets add the goal
    f.write('(:goal (and\n   ')
    if homogen:
      # all the robots want that all the steps an every machine are completed
      for val in statasign['robot' + str(i)]:
        try:
          for prod in stepLoc[val]:
            f.write('(step-completed ' + prod + ')\n   ')
        except KeyError:
          pass
    else:
      pass
      # Todo: implement this
    f.write('))\n   (:metric minimize (total-cost))\n   )')


if __name__ == "__main__":
  agentNum = 1
  statAsign = defaultdict()
  statAsign['robot1'] = ['bs0', 'rs0', 'rs1', 'cs0', 'cs1', 'ds0']
  generateDomain(agentNum=agentNum)
  generateProblems(agentNum=agentNum, statasign=[])
