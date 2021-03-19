#!/usr/bin/env python

import sys
import json
from utils import *
from parsers import Parser

if __name__ == '__main__':


    file1 = open('CIFP/FAACIFP18', 'r')
    Lines = file1.readlines()


    parser = Parser()

    parser.parse(Lines[124070:228258])

    print(json.dumps(parser.airports_data,sort_keys=True, indent=4))
