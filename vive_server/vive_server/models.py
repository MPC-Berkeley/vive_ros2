from pydantic import BaseModel, Field


class ViveTrackerMessage(BaseModel):
    valid: int = Field(default=0)
    x: float = Field(default=0)
    y: float = Field(default=0)
    z: float = Field(default=0)
    qx: float = Field(default=0)
    qy: float = Field(default=0)
    qz: float = Field(default=0)
    qw: float = Field(default=1)
    device_name: str = Field(default="tracker_1")
    vel_x: float = Field(default=0)
    vel_y: float = Field(default=0)
    vel_z: float = Field(default=0)
    p: float = Field(default=0)
    q: float = Field(default=0)
    r: float = Field(default=0)

    def __repr__(self):
        return f"{self.device_name} -> " \
               f"x: {round(self.x, 5)}, y: {round(self.y, 5)}, z: {round(self.z, 5)} | " \
               f"qx: {round(self.qx, 5)}, qy: {round(self.qy, 5)}, qz: {round(self.qz, 5)}, qw: {round(self.qz, 5)} | " \
               f"vel_x: {round(self.vel_x, 5)}, vel_y: {round(self.vel_y, 5)}, vel_z: {round(self.vel_z, 5)}" \
               f"p: {round(self.p, 5)}, q: {round(self.q, 5)}, r: {round(self.r, 5)}"

    def __str__(self):
        return self.__repr__()
