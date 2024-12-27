from functools import wraps
# from threading import RLock, _RLock
import threading

def synchronized(lock_attr_name: str):
    """ Synchronization decorator wrapping whole methods accessing specified attribute """
    def wrapper(f):
        @wraps(f)
        def wrapped(self, *args, **kwargs):
            lock = getattr(self, lock_attr_name)
            dummy_rlock = threading.RLock()
            if not isinstance(lock, type(dummy_rlock)):
                raise TypeError(f"The attribute '{lock_attr_name}' must be an instance of RLock")
            with lock:
                return f(self, *args, **kwargs)
        return wrapped
    return wrapper