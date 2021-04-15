a #!/usr/bin/env python

import sys
import json
from utils import *
from plan_visualizer import Visualizer
from plan import *

if __name__ == '__main__':

    
    CIFP_parsed_filename = 'CIFP_parsed/CIFP_parsed'
    

    visualizer = Visualizer(CIFP_parsed_filename)
