#!/usr/bin/env python
### A simple script to generate tray box objects in simulation

__version__ = "0.1"

import sys, os, getopt
from shutil import copyfile
from xml.etree import ElementTree as et
from IPython import embed

def generate_cube(tray_name, colors):
    """
    Takes input: alphabet, red, green, blue channels
    """
    urdf_name = '{}.urdf'.format(tray_name)
    copyfile('templates/tray_textured2.urdf', urdf_name)

    tree = et.parse(urdf_name)
    element = tree.find('.//color')
    element.set('rgba', '{} {} {} 1'.format(*colors))
    element = tree.findall('.//mesh')
    for e in element:
        e.set('scale', '0.2, 0.2, 0.2')
    tree.write(urdf_name)

def run(argv):

    # Assume 200 users
    r, g, b = 255, 255, 255
    n = 'tray'
    # One minute of data
    try:
        opts, args = getopt.getopt(argv,
                                   'r:g:b:n:', ['red=', 'green=', 'blue=', 'name='])
    except getopt.GetoptError:
        sys.exit(2)
    for opt, arg in opts:
        if opt in ('-r', '--red'):
            r = arg
        elif opt in ('-g', '--green'):
            g = arg
        elif opt in ('-b', '--blue'):
            b = arg
        elif opt in ('-n', '--name'):
            n = arg

    generate_cube(n, (r, g, b))

if __name__ == '__main__':
    run(sys.argv[1:])
