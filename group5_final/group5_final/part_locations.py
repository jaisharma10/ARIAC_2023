#!/usr/bin/env python3

"""
Holds classes useful to identfying and storing parts in the workcell
Potential Part Locations:
    - Bins
    - Conveyors
    - AGVs
"""

from ariac_msgs.msg import BinParts, BinInfo, PartLot, ConveyorParts
from group5_final.store_and_submit import PartClass

class BinPartsClass:
    def __init__(self, msg:BinParts):
        self.number_of_bins = len(msg.bins)
        self.bins = []
        for i in range(self.number_of_bins):
            self.bins.append(BinInfoClass(msg.bins[i]))
        
class BinInfoClass:
    def __init__(self, msg:BinInfo):
        self.bin_number = msg.bin_number
        self.number_of_parts = len(msg.parts)
        self.parts = []
        for i in range(self.number_of_parts):
            self.parts.append(PartLotClass(msg.parts[i]))

class PartLotClass:
    def __init__(self, msg:PartLot):
        self.quantity = msg.quantity
        self.part = PartClass(msg.part)

class ConveyerPartsClass:
    def __init__(self, msg:ConveyorParts):
        self.number_of_parts = len(msg.parts)
        self.parts = []
        for i in range(self.number_of_parts):
            self.parts.append(PartLotClass(msg.parts[i]))