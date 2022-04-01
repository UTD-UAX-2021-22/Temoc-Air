import math
from pathlib import Path
from turtle import color
import pandas as pd
import plotly
import plotly.express as px
import plotly.graph_objects as go
import numpy as np
from scipy.spatial.transform import Rotation
import plotly.figure_factory as ff
import Utils as Utils
import utm
import json

config = dict({'scrollZoom': True})
data_dir = "flight_test_data"
# df = pd.read_json(Path('data/geo_tracker_log.jsonl'), lines=True)
vehicle_positions = pd.read_json(Path(f'{data_dir}/pix2world_vpos.jsonl'), lines=True)
vehicle_atts = pd.read_json(Path(f'{data_dir}/v_att.jsonl'), lines=True)
centroids = pd.read_json(Path(f'{data_dir}/centroids.jsonl'), lines=True)

with open('camera.json') as vf:
    cam = Utils.CameraInfo(**json.load(vf))

corners = np.array([[  696695.51467665, -3915480.2016415 ],
 [  696696.31260067, -3915434.48860487],
 [  696745.07286831, -3915435.33971851],
 [  696744.27494429, -3915481.05275513]])

def rowToObjects(f):
    dv = Utils.DummyVehicle(
        location=Utils.DummyLocation(global_relative_frame=Utils.DummyGPSCoords(lat=f["lat"],lon=f["lon"],alt=f["z"])),
        attitude=Utils.DummyAttitude(pitch=f["pitch"],roll=f["roll"],yaw=f["yaw"]),
        heading=f["heading"]
    )
    px = np.array([[f["x_p"], f["y_p"]]])
    return (dv, cam, px)


# print(vecs_df)
df = vehicle_positions.merge(vehicle_atts, left_on='mission_time', right_on='mission_time', how='inner')
df = df.merge(centroids, left_on='mission_time', right_on='mission_time', how='inner', suffixes=("","_p"))
# print(df)
df["lat"], df["lon"], *_ = utm.to_latlon(df["x"].to_numpy().flatten(),df["y"].to_numpy().flatten(),df["zl"][0],df["zn"][0])
# print(df)

df["r_i"] = (df["mission_time"].diff() < 0).cumsum()
df = df[df["r_i"] == 0]
temp = df.apply(lambda row: Utils.pixCoordToWorldPosition(*rowToObjects(row)), axis=1)
tdf = pd.DataFrame(columns=["x_c", "y_c", "z_c"])
# print(temp.values[0])
for r in temp:
    v = r.flatten()
    tdf.loc[len(tdf.index)] = v 
df = pd.concat([df,tdf], axis=1)

print(df)
xm, ym = df[["x", "y"]].mean()[["x", "y"]]

corners = np.array([[ 696776.46083925, 6084453.23110203],
 [ 696754.29534341, 6084413.24348902],
 [ 696711.64215612, 6084436.88653681],
 [ 696733.80765196, 6084476.87414982]])

fig = px.scatter(x=df["x_c"]-xm, y=df["y_c"]-ym, color=df["mission_time"])
fig.add_trace(go.Scatter(x=df["x"]-xm, y=df["y"]-ym, text=df["mission_time"]))
# fig.add_trace(go.Scatter(x=corners[:,0]-xm, y=corners[:,1]-ym, fill="toself", text=[0, 1, 2, 3]))
fig.update_yaxes(
    scaleanchor = "x",
    scaleratio = 1,
  )
fig.show(config=config)
# px.scatter(df, x="mission_time", y="z_v")
print(df.describe())
# print(df)
# print(vecs_df)
# vehicle_rot =  Rotation.from_euler('ZYX', [-vehicle.heading*math.pi / 180.0, -vehicle.attitude.pitch, -vehicle.attitude.roll])
# print(p2w_v_pos)
