from .bullet import BulletWorld


def get_world(
        physics,
        display=True,
        data_dir='.',
        verbose=False,
        key=None,
        camera_params={}):

    if physics == 'bullet':
        return BulletWorld(
                display,
                data_dir,
                verbose,
                key=key,
                camera_params=camera_params)
    else:
        raise ValueError('Unrecognized simulato')
