from pydantic import BaseModel, Field, model_validator
from typing import List, Literal, Optional


class LinkRef(BaseModel):
    link: str = Field(alias='@link')


class Origin(BaseModel):
    rpy: List[float] = Field(alias='@rpy')
    xyz: List[float] = Field(alias='@xyz')

    @model_validator(mode='before')
    @classmethod
    def split_space_separated_values(cls, values):
        for key in ['@rpy', '@xyz']:
            if key in values:
                values[key] = [float(v) for v in values[key].split()]
        return values


class Axis(BaseModel):
    xyz: List[float] = Field(alias='@xyz')

    @model_validator(mode='before')
    @classmethod
    def split_xyz(cls, values):
        if '@xyz' in values:
            values['@xyz'] = [float(v) for v in values['@xyz'].split()]
        return values


class Limit(BaseModel):
    effort: float = Field(alias='@effort')
    lower: float = Field(alias='@lower')
    upper: float = Field(alias='@upper')
    velocity: float = Field(alias='@velocity')


class Dynamics(BaseModel):
    damping: float = Field(alias='@damping')
    friction: float = Field(alias='@friction')


class Joint(BaseModel):
    name: str = Field(alias='@name')
    type: Literal['prismatic', 'revolute', 'fixed'] = Field(alias='@type')
    parent: LinkRef
    child: LinkRef
    origin: Origin
    limit: Optional[Limit] = None
    dynamics: Optional[Dynamics] = None
