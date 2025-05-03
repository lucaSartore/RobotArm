from pydantic import BaseModel, Field, root_validator
from typing import List


class LinkRef(BaseModel):
    link: str = Field(alias='@link')


class Origin(BaseModel):
    rpy: List[float] = Field(alias='@rpy')
    xyz: List[float] = Field(alias='@xyz')

    @root_validator(pre=True)
    def split_space_separated_values(cls, values):
        for key in ['@rpy', '@xyz']:
            if key in values:
                values[key] = [float(v) for v in values[key].split()]
        return values


class Joint(BaseModel):
    name: str = Field(alias='@name')
    type: str = Field(alias='@type')
    parent: LinkRef
    child: LinkRef
    origin: Origin
