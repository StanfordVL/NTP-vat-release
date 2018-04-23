"""Define the XML file format of the world configuration specification."""


def _add_attrib(node, name, required, type=None, default=None, help=''):
    attrib = {
            'name': name,
            'required': required,
            'type': type,
            'default': default
            }
    node['attrib'].append(attrib)
    return attrib


def _add_elem(node, name, required, type=None, default=None, help=''):
    elem = {
            'name': name,
            'required': required,
            'type': type,
            'default': default,
            'attrib': [],
            'elem': []
            }
    node['elem'].append(elem)
    return elem


default_pose = {'rpy': [0.0, 0.0, 0.0], 'xyz': [0.0, 0.0, 0.0]}

# L0 Element: root
spec_root = {'elem': [], 'attrib': [], 'type': None, 'default': None} 
world = _add_elem(spec_root, 'world', required=True, type=None, default=None,
        help='')

# L1 Element: world
name = _add_attrib(world, 'name', required=True, type=str, default=None,
        help='The world name.')
body = _add_elem(world, 'body', required=None, type=None, default=None,
        help='A body.')
robot = _add_elem(world, 'robot', required=None, type=None, default=None,
        help='')
gravity = _add_elem(world, 'gravity', required=False, type=float,
        default=[0.0, 0.0, 0.0], help='The gravity.')
gui = _add_elem(world, 'gui', required=False, type=None, default=None, help='')

# L2 Element: body
name = _add_attrib(body, 'name', required=True, type=str, default='',
        help='')
model = _add_elem(body, 'model', required=True, type=None, default=None,
        help='')
pose = _add_elem(body, 'pose', required=False, type=None, default=default_pose,
        help='')
initial = _add_elem(body, 'initial', required=False, type=float, default=None,
        help='')
fixed = _add_elem(body, 'fixed', required=False, type=bool, default=False,
        help='')

filename = _add_attrib(model, 'filename', required=False, type=str,
        default=None, help='')

rpy = _add_attrib(pose, 'rpy', required=True, type=float,
        default=[0.0, 0.0, 0.0], help='')
xyz = _add_attrib(pose, 'xyz', required=True, type=float,
        default=[0.0, 0.0, 0.0], help='')

# L2 Element: robot
name = _add_attrib(robot, 'name', required=True, type=str, default=None,
        help='')
type = _add_attrib(robot, 'type', required=True, type=str, default='robot',
        help='')
body = _add_elem(robot, 'body', required=None, type=None, default=None,
        help='')
pose = _add_elem(robot, 'pose', required=False, type=None, default=default_pose,
        help='')
fixed = _add_elem(robot, 'fixed', required=False, type=bool, default=False,
        help='')

rpy = _add_attrib(pose, 'rpy', required=True, type=float,
        default=[0.0, 0.0, 0.0], help='')
xyz = _add_attrib(pose, 'xyz', required=True, type=float,
        default=[0.0, 0.0, 0.0], help='')

# L3 Element: body
name = _add_attrib(body, 'name', required=True, type=str, default='',
        help='')
model = _add_elem(body, 'model', required=True, type=None, default=None,
        help='')
pose = _add_elem(body, 'pose', required=False, type=None, default=default_pose,
        help='')
initial = _add_elem(body, 'initial', required=False, type=float, default=None,
        help='')
fixed = _add_elem(body, 'fixed', required=False, type=bool, default=False,
        help='')

filename = _add_attrib(model, 'filename', required=False, type=str,
        default=None, help='')

rpy = _add_attrib(pose, 'rpy', required=True, type=float,
        default=[0.0, 0.0, 0.0], help='')
xyz = _add_attrib(pose, 'xyz', required=True, type=float,
        default=[0.0, 0.0, 0.0], help='')

# L2 Element: gui
name = _add_attrib(gui, 'name', required=True, type=str, default=None,
        help='')
camera = _add_elem(gui, 'camera', required=False, type=None, default=None,
        help='')

name = _add_attrib(camera, 'name', required=True, type=str, default=None,
        help='')
frame = _add_elem(camera, 'frame', required=False, type=str, default=None,
        help='')
pose = _add_elem(camera, 'pose', required=False, type=None, default=None,
        help='')

rpy = _add_attrib(pose, 'rpy', required=True, type=float,
        default=[0.0, 0.0, 0.0], help='')
xyz = _add_attrib(pose, 'xyz', required=True, type=float,
        default=[0.0, 0.0, 0.0], help='')

