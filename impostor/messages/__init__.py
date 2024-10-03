import sys
import pathlib

# add this folder to the system path so the generated file can import its dependencies
sys.path.append(str(pathlib.Path(__file__).parent.absolute()))

from .messages import *  # noqa: F403
