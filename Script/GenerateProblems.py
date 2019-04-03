"""
Author: Ben Bausch
"""
from collections import defaultdict
import Domain2 as dm2
import os

#---------------------------do not change this!--------------------------------------------------------------------------------------------------------------------------
path=os.getcwd() + "/Problems/"

def generateDomains():
  #---------------------------change products here to make problems more complex or easier--------------------------------------------------------------------------------
  small_product=['silver_base', 'gray_cap', 'gate2_delivery']
  intermediate_product=['silver_base', 'blue_ring', 'orange_ring', 'black_cap', 'gate2_delivery']
  big_product=['blue_base', 'gray_ring', 'purple_ring', 'yellow_ring', 'black_ring', 'green_ring', 'red_ring', 'brown_ring', 'mint_ring', 'gray_cap', 'gate2_delivery' ]
  max_product=[small_product, intermediate_product, big_product]

  #---------------------------change setting to make problems more complex or easier--------------------------------------------------------------------------------------
  stations=[1,2,2,1]
  max_agent_num = 5
  privateVar = True
  homogen = True 
  world_width=30
  world_length=30

  #---------------------------generate the different problems-------------------------------------------------------------------------------------------------------------
  for agent_count in range(1,max_agent_num+1):
    for prod_len in range(1, len(max_product)+1):
      prod = max_product[:prod_len]
      new_path = path + "problem_" + str(agent_count) + "_" + str(prod_len)    
      os.mkdir(new_path)
      os.chmod(new_path, 0o777)
      print(new_path[:11])
      print(prod)
      dm2.generateDomains(new_path, privateVar, homogen, stations, agent_count, prod)
      dm2.generateProblems(new_path, stations, agent_count, prod, width=world_width, length=world_length)

if __name__ == "__main__":
  generateDomains()
     
   




