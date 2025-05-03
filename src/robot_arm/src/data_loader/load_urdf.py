from typing import List
from pydantic import TypeAdapter
import xmltodict
from constants.path import URDF_FILE_PATH
from internal_types.urdf_model import Joint
import os

def load_urdf() -> List[Joint]:

    with open(URDF_FILE_PATH) as f:
        xml = xmltodict.parse(
            f.read()
        )

    joins = TypeAdapter(List[Joint]).validate_python(xml["robot"]["joint"])


    for (prev, next) in zip(joins[:-1], joins[1:]):
        assert prev.child == next.parent, "The joins should be loaded in order"

    return joins

