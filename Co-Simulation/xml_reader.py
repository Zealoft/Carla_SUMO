'''
read from a .sumo.tr file
get a vehicle's whole route
'''


try:
    import xml.etree.cElementTree as ET
except ImportError:
    import xml.etree.ElementTree as ET

import argparse
import sys
import os

class XML_Tree():
    def __init__(self, file_name):
        self.file = file_name
        self.tree = ET.ElementTree(file=file_name)
        self.root = self.tree.getroot()
    
    def read_offset(self):
        offset = []
        for elem in self.tree.iter():
            if (elem.tag == 'location'):
                offsets = elem.attrib['netOffset'].split(',')
                offset.append(float(offsets[0]))
                offset.append(float(offsets[1]))
        print('read offset: ', offset)
        return offset

    def read_routes(self):
        routes = []
        for elem in self.tree.iter():
            if (elem.tag == 'route'):
                edges = elem.attrib['edges'].split(' ')

                routes.append(edges)
            # print(elem.tag, elem.attrib)
        print('routes list len: ', len(routes))
        # print(routes)
        return routes
    def read_single_route(self, vehicle_id):
        vehicle_route = []
        i = 0
        for elem in self.tree.iter():
            if(elem.tag == 'vehicle' and elem.attrib['id'] == str(vehicle_id)):
                print(elem.tag, elem.attrib)
                # i += 1
                # if i > 10:
                #     break
        # for child in self.root:
        #     print(child.attrib)
        #     i += 1
        #     if i > 10:
        #         break
        # print(self.root)

def read_routes(path):
    pass

if __name__ == '__main__':
    argparser = argparse.ArgumentParser(
        description='SUMO Simulation XML Reader')
    argparser.add_argument(
        '-S', '--source',
        default='simulations/Town06/Town06.sumocfg',
        help='source of the sumo config file',
    )
    args = argparser.parse_args()
    
    tree = XML_Tree(args.source)
    tree.read_routes()