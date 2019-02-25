"""
Author: Ben Bausch

"""
import numpy as np


def generateRadnomLayout(bs_num=1, rs_num=2, cs_num=2, ds_num=1, width=30, length=30, agentNum=3):
  """

  inputs:
  :param bs_num: number of base stations
  :param rs_num: number of ring stations
  :param cs_num: number of cap stations
  :param ds_num: number of delivery stations
  :param width: width of the world
  :param length: length of the world

  :return: returns a list of positions ordered by type, station_pos, output_pos, input_pos

  """
  counters = [bs_num, rs_num, cs_num, ds_num]
  positions = []
  world = np.zeros((length,width))
  for i in range(0,agentNum):
    if agentNum < width:
      world[0][i]= 666
      positions.append([i, (0, i), None, None, 666])
  #now we have to fill in the stations at random positions
  for i in range(0,4):
    # determine a random position
    j = counters[i]
    while counters[i] > 0:
      xpos = np.random.randint(1,width-1) # we form a barrier of 0 to be 100 per cent able to access in/out put
      ypos = np.random.randint(1,length-1)
      #if position not occupied
      if world[ypos][xpos] == 0:
        world[ypos][xpos] = i+1
        #to mark in/out-put
        world[ypos+1][xpos] = i+1
        world[ypos][xpos+1] = i+1
        if i+1 != 1 and i+1 != 4:
          positions.append([i + 1, (ypos, xpos), (ypos + 1, xpos), (ypos, xpos + 1), j - counters[i]])
        elif i+1 == 1:
          positions.append([i + 1, (ypos,  xpos), (ypos + 1, xpos), (ypos, xpos + 1), j - counters[i]])
        elif i+1 == 4:
          positions.append([i + 1, (ypos, xpos), None, (ypos, xpos + 1), j - counters[i]])
      counters[i] -= 1


  return positions



if __name__ == "__main__":
  A = generateRadnomLayout(length=10, width=6)
  print(A)
