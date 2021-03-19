#!/usr/bin/env python

def find_end_linum(Lines,code_type,code):

    if code_type == "section":
        pos = 4
    elif code_type == "airport":
        pos = (6,10)
    elif code_type == "subsec":
        pos = 12
    elif code_type == "SID" or code_type=="STAR" or code_type=="IAP":
        pos = (13,19)
    elif code_type == "transition":
        pos = (20,25)

    linum = 0

    for line in Lines:
        if type(pos)==int:
            if line[pos] == code:
                linum += 1
        elif type(pos)==tuple:
            if line[pos[0]:pos[1]] == code:
                linum += 1

    return linum

