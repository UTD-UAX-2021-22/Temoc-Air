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
data_dir = "data"
df = pd.read_json(f"{data_dir}/logo_seek.jsonl", lines=True)
pdf = pd.read_json(f"{data_dir}/main_dump.jsonl", lines=True)
pdf["x"], pdf["y"], *_ = utm.from_latlon(pdf["lat"].to_numpy(), pdf["lon"].to_numpy())
cpdf = df.merge(pdf, left_on='mission_time', right_on='mission_time', how='inner', suffixes=("","_p"))
cpdf["r_i"] = (cpdf["mission_time"].diff() < 0).cumsum()
cpdf = cpdf[cpdf["r_i"] == 2]
print(cpdf)

xm, ym = cpdf[["x", "y"]].mean()[["x", "y"]]
fig = px.scatter(x=cpdf["x_lr"]-xm+cpdf["x"], y=cpdf["y_lr"]+cpdf["y"]-ym, color=cpdf["mission_time"])
fig.update_yaxes(
    scaleanchor = "x",
    scaleratio = 1,
  )
fig.add_trace(go.Scatter(x=cpdf["x"]-xm, y=cpdf["y"]-ym, text=cpdf["mission_time"]))

fig.update_layout(dragmode='pan')
fig.show(config=config)


