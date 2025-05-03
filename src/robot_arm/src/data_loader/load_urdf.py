from typing import List
from pydantic import TypeAdapter
import xmltodict
from constants.path import URDF_FILE_PATH
from internal_types.urdf_model import Joint
import os

def load_urdf() -> List[Joint]:

    print(os.getcwd())
    with open(URDF_FILE_PATH) as f:
        xml = xmltodict.parse(
            f.read()
        )

    return TypeAdapter(List[Joint]).validate_python(xml["robot"]["joint"])

