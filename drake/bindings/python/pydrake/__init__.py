from . import rbtree

import warnings
warnings.warn("This API is completely experimental and likely to change very soon. Use with caution.", FutureWarning)

try:
    from autogenerated_path import getDrakePath
except ImportError:
    from insource_path import getDrakePath