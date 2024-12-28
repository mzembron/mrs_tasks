from functools import wraps
# from threading import RLock, _RLock
import threading

def synchronized(lock_attr_name: str):
    """ Synchronization decorator wrapping whole methods accessing specified attribute """
    def wrapper(f):
        @wraps(f)
        def wrapped(self, *args, **kwargs):
            lock = getattr(self, lock_attr_name)
            if not isinstance(lock, type(threading.RLock())):
                raise TypeError(f"The attribute '{lock_attr_name}' must be an instance of RLock")
            with lock:
                return f(self, *args, **kwargs)
        return wrapped
    return wrapper