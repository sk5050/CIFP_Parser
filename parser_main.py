#!/usr/bin/env python

import sys
import json
from utils import *
from parsers import Parser

if __name__ == '__main__':


    CIFP = open('CIFP/FAACIFP18', 'r')
    Lines = CIFP.readlines()


    parser = Parser()

    parser.parse(Lines[23029:232000])


    with open('CIFP_parsed/CIFP_parsed', 'w') as outfile:
        json.dump(parser.CIFP_parsed, outfile, sort_keys=True, indent=4)
