from functools import wraps 

def synchronized(lock_attr_name:str):
    """ Synchronization decorator wrapping whole methods accessing specified attribute """
    def wrapper(f):
        @wraps(f)
        def wrapped(self, *args, **kwargs):
            lock = getattr(self, lock_attr_name)
            with lock:
                return f(self, *args, **kwargs)
        return wrapped
    return wrapper