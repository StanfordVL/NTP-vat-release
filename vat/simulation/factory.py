from .bullet import BulletWorld

def get_world(
        physics,
        display=True,
        data_dir='.',
        verbose=False,
        key=None):

    if physics == 'bullet':
        return BulletWorld(
                display,
                data_dir,
                verbose,
                key=key)
    else:
        raise ValueError('Unrecognized simulator name \'{}\'.'.format(name))
