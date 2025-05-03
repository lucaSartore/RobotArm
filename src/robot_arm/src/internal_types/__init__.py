from pydantic import BaseModel, Field
from typing import List, Optional, Literal

class Origin(BaseModel):
    rpy: str
    xyz: str

class Axis(BaseModel):
    xyz: str

class Limit(BaseModel):
    effort: float
    lower: float
    upper: float
    velocity: float

class Dynamics(BaseModel):
    damping: float
    friction: float

class Inertia(BaseModel):
    ixx: float
    ixy: float
    ixz: float
    iyy: float
    iyz: float
    izz: float

class Mass(BaseModel):
    value: float

class Inertial(BaseModel):
    mass: Mass
    origin: Origin
    inertia: Inertia

class GeometryCylinder(BaseModel):
    length: float
    radius: float

class GeometryBox(BaseModel):
    size: str

class GeometrySphere(BaseModel):
    radius: float

class Geometry(BaseModel):
    cylinder: Optional[GeometryCylinder] = None
    box: Optional[GeometryBox] = None
    sphere: Optional[GeometrySphere] = None

class Visual(BaseModel):
    geometry: Geometry
    origin: Optional[Origin] = None

class Collision(BaseModel):
    geometry: Geometry
    origin: Optional[Origin] = None

class Link(BaseModel):
    name: str
    visual: Optional[Visual] = None
    collision: Optional[Collision] = None
    inertial: Optional[Inertial] = None

class Joint(BaseModel):
    name: str
    type: Literal["fixed", "revolute", "prismatic"]
    parent: str
    child: str
    origin: Optional[Origin] = None
    axis: Optional[Axis] = None
    limit: Optional[Limit] = None
    dynamics: Optional[Dynamics] = None

class Plugin(BaseModel):
    filename: str
    name: str

class Gazebo(BaseModel):
    plugin: Plugin

class Robot(BaseModel):
    name: str
    gazebo: Optional[Gazebo]
    links: List[Link]
    joints: List[Joint]
