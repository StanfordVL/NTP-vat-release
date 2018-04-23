import numpy as np
import xml.etree.cElementTree as ET

from spec import spec_root


def parse_world_from_file(path):
    """Parse the world description from the .xml file."""
    xml_root = ET.ElementTree(file=path).getroot()
    w = _parse_element(spec_root, [xml_root])
    return w


def _parse_text(text, dtype):
    """Parse the text of an element or an attribute."""
    if dtype == str:
        return text
    elif dtype == float:
        ret = [float(word) for word in text.split(' ')]
        if len(ret) == 1:
            ret = ret[0]
        return ret
    elif dtype == bool:
        return bool(text)
    else:
        raise ValueError('Unrecognized type {} in Element{}'.format)


def _parse_element(spec_node, xml_node):
    """Parse an element in the world description."""
    descr_node = {}

    if spec_node['type'] is not None:
        return _parse_text(xml_node.text, spec_node['type'])

    # Parse elements
    for spec_elem in spec_node['elem']:
        # Look for the coresponding element in xml data
        key = spec_elem['name']
        found = False
        if spec_elem['required'] is None:
            descr_elem = []
            for xml_elem in xml_node:
                if xml_elem.tag == key:
                    descr_elem.append(_parse_element(spec_elem, xml_elem))
                    found = True
        else:
            for xml_elem in xml_node:
                if xml_elem.tag == key:
                    descr_elem = _parse_element(spec_elem, xml_elem)
                    found = True
                    break
        # Retrive element values
        if found:
            descr_node[key] = descr_elem
        else:
            descr_node[key] = spec_elem['default']

    # Parse attributes
    for spec_attrib in spec_node['attrib']:
        key = spec_attrib['name']
        if xml_node.attrib.has_key(key):
            descr_node[key] = _parse_text(xml_node.attrib[key],
                    spec_attrib['type'])
        else:
            descr_node[key] = spec_attrib['default']

    return descr_node
