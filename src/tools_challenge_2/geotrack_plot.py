import math
from pathlib import Path
import pandas as pd
import plotly
import plotly.express as px
import plotly.graph_objects as go
import numpy as np
from scipy.spatial.transform import Rotation
import plotly.figure_factory as ff
data_dir = "data_real_flight"
config = dict({'scrollZoom': True})

df = pd.read_json(Path(f'{data_dir}/geo_tracker_log.jsonl'), lines=True)
p2w_v_pos = pd.read_json(Path(f'{data_dir}/pix2world_vpos.jsonl'), lines=True)
att_df = pd.read_json(Path(f'{data_dir}/v_att.jsonl'), lines=True)
att_df["heading"] = -att_df["heading"] * math.pi / 180.0
rotators = Rotation.from_euler('ZYX', att_df[["heading", "pitch", "roll"]].to_numpy())

camm_offset = rotators.apply(np.array([0.13, 0, 0.2]))

cam_rot = Rotation.from_euler('ZXY', [90, -45, 0], degrees=True)
vecs = cam_rot.apply(rotators.apply(np.array([1, 0, 0])))
vecs_df = pd.DataFrame(vecs, columns=["vx", "vy", "vz"])
vecs_df["mission_time"] = att_df["mission_time"]
# print(vecs_df)
vecs_df = vecs_df.merge(p2w_v_pos, left_on='mission_time', right_on='mission_time', how='inner')
# print(vecs_df)
# vehicle_rot =  Rotation.from_euler('ZYX', [-vehicle.heading*math.pi / 180.0, -vehicle.attitude.pitch, -vehicle.attitude.roll])
# print(p2w_v_pos)
# quivs = ff.create_quiver(vecs_df["x"], vecs_df["y"], vecs_df["vx"], vecs_df["vy"], arrow_scale=0.2, scale=1)
# quivs = ff.create_quiver(vecs_df["x"], vecs_df["y"], camm_offset[:,0], camm_offset[:,1], arrow_scale=0.2, scale=1)
# quivs.add_trace(go.Scatter(x=vecs_df["x"], y=vecs_df["y"], text=vecs_df["mission_time"]))
# quivs.update_layout(dragmode='pan')
# quivs.update_yaxes(
#     scaleanchor = "x",
#     scaleratio = 1,
# )
# quivs.show(config=config)

means = p2w_v_pos[["x", "y"]].mean()
xm, ym = means[["x", "y"]]
print(means)
print(xm)
print(ym)

fig = px.scatter(df, x="lat", y="lon", color='time')

px.scatter(vecs_df, x="mission_time", y="vz").show(config=config)

# cones = go.Figure(data=go.Cone(x=vecs_df["x"]-xm, y=vecs_df["y"]-ym, z=np.ones_like(vecs_df["x"]), u=vecs_df["vx"], v=vecs_df["vy"], w=vecs_df["vz"],     colorscale='Blues',
#     sizemode="absolute",
#     sizeref=40))
# cones.update_yaxes(
#     scaleanchor = "x",
#     scaleratio = 1,
#   )
# cones.update_zaxes(
#     scaleanchor = "x",
#     scaleratio = 1,
# #   )
# cones.update_layout(scene_aspectmode='data',
#                   scene_aspectratio=dict(x=1, y=1, z=1))
# # cones.update_layout(scene_aspectmode='cube')
# cones.show(config=config)

v_pos = go.Scatter(x=p2w_v_pos["x"], y=p2w_v_pos["y"], text=p2w_v_pos["mission_time"])
fig.add_trace(v_pos)
fig.add_trace(go.Scatter(x=(p2w_v_pos["x"]+camm_offset[:,0]), y=(p2w_v_pos["y"]+camm_offset[:,1]), text=p2w_v_pos["mission_time"]))
# fig = go.Figure(go.Scatter3d(
#     x=df["lat"], y=df["lon"], z=np.zeros((len(df["lat"]))),
#     marker=dict(
#         size=4,
#         color=df['time'],
#         colorscale='Viridis',
#     )
# ))

# fig2 = go.Figure(go.Scatter3d(
#     x=df2["x"], y=df2["y"], z=np.zeros((len(df2["x"]))),
#     marker=dict(
#         size=0,
#         color=df['time'],
#         colorscale='Viridis',
#     ),
#         line=dict(
#         # color='darkblue',
#                 colorscale='Viridis',
#                         color=df['time'],
#         width=2
#     )
# ))



fig.update_yaxes(
    scaleanchor = "x",
    scaleratio = 1,
  )
fig.update_layout(dragmode='pan')
fig.show(config=config)

# fig2.update_layout(dragmode='pan')
# fig2.show(config=config)
# # px.scatter()