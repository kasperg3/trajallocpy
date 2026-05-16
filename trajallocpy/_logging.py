"""Central library logger.

Following library best practice the package never configures handlers itself
(only a :class:`logging.NullHandler`), so importing ``trajallocpy`` stays
silent unless the application opts in via ``logging.basicConfig`` or attaches
its own handler.
"""

import logging

logger = logging.getLogger("trajallocpy")
logger.addHandler(logging.NullHandler())
