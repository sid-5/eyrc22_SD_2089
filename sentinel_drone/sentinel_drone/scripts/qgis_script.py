#!/usr/bin/env python3

from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from sentinel_drone.msg import Geolocation
import rospy
import time
import pandas as pd

df = pd.read_csv("/home/sid/Downloads/YellowBox.csv")
for i in range(len(df)):
    lat,lon = df.iloc[i][1], df.iloc[i][2]
    canvas = iface.mapCanvas()
    pnt = QgsPointXY(lat, lon)
    m = QgsVertexMarker(canvas)
    m.setCenter(pnt)
    m.setColor(QColor('Black'))
    m.setIconType(QgsVertexMarker.ICON_CIRCLE)
    m.setIconSize(12)
    m.setPenWidth(1)
    m.setFillColor(QColor(0, 200, 0))
    print(lat,lon)