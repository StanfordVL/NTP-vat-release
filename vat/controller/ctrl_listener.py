# from multiprocessing import Process
import time
import warnings


def get_ctrl_listener(name=None):
    """Get the controller listener."""
    if name == None:
        ctrl_listener = None
    elif name == 'keyboard':
        ctrl_listener = KeyListener()
    else:
        raise ValueError('Unrecognized controller listener type {}.'\
                .format(name))
    return ctrl_listener


class CrtlListener(object):
    """Controller listener."""

    def __init__(self):
        self._events_fetcher = self._default_events_fetcher
        self._handlers = {}

    def listen(self, debug=True):
        """Listen to the events."""
        if len(self._handlers) == 0:
            warnings.warn('No event handler has been added.')

        events = self._events_fetcher()
        if debug and len(events) > 0:
            print events

        for e in events:
            if self._handlers.has_key(e):
                self._handlers[e]()

    def _default_events_fetcher(self):
        """The default event fetcher."""
        raise NotImplementedError

    @property
    def events_fetcher(self):
        return self._events_fetcher

    @events_fetcher.setter
    def events_fetcher(self, value):
        self._events_fetcher = value

    @property
    def handlers(self):
        return self._handlers


class KeyListener(CrtlListener):
    """Keyboard listener."""

    def __init__(self):
        super(KeyListener, self).__init__()
        self._keys = None
        self._key_acts = None
        self._modifiers = None
        self._records = {}

    def listen(self, debug=False):
        """Listen to the events."""
        if len(self._handlers) == 0:
            warnings.warn('No event handler has been added.')

        events = self._events_fetcher()
        events = self._process_events(events)
        if debug and len(events) > 0:
            print events

        for e in events:
            if self._handlers.has_key(e):
                handler, delay = self._handlers[e]
                t = time.time()
                if t >= self._records[e] + delay:
                    self._records[e] = t
                    handler()

    def _default_events_fetcher(self):
        """The default event listener."""
        raise NotImplementedError

    def _process_events(self, events):
        """Process the events to be tuples of (key, key_act, modifiers)."""
        # Seperate modifiers and keys
        modifiers = []
        key_act_tuples = []
        for e in events:
            if e == []:
                pass
            elif e[0] in self.modifiers:
                modifiers.append(e[0])
            elif e[0] in self.keys:
                key_act_tuples.append(e)
            else:
                raise ValueError
        # Create processed events
        new_events = []
        new_events.append((None, None, None)) # The default event
        for key, key_act in key_act_tuples:
            new_events.append((key, key_act, None)) # None for any modifiers
            new_events.append((key, key_act, tuple(set(modifiers))))
        return new_events


    def on_key_event(self, key, key_act, modifiers=None, delay=0.0,
            handler=None):
        """Add a keyboard event to the listener."""
        if modifiers is not None:
            modifiers = tuple(set(modifiers))
        self._handlers[(key, key_act, modifiers)] = (handler, delay)
        self._records[(key, key_act, modifiers)] = 0.0

    @property
    def keys(self):
        if self._keys is None:
            keys = []
            for i in xrange(128):
                keys.append(str(unichr(i)))
            keys.append('LEFT')
            keys.append('RIGHT')
            keys.append('UP')
            keys.append('DOWN')
            keys.append('ENTER')
            self._keys = keys
        return self._keys

    @property
    def key_acts(self):
        if self._key_acts is None:
            key_acts = []
            key_acts.append('DOWN')
            key_acts.append('TRIGGERED')
            key_acts.append('RELEASED')
            self._key_acts = key_acts
        return self._key_acts

    @property
    def modifiers(self):
        if self._modifiers is None:
            modifiers = []
            modifiers.append('SHIFT')
            modifiers.append('CTRL')
            modifiers.append('OPTION')
            self._modifiers = modifiers
        return self._modifiers
