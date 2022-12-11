import pandas as pd
import time

while True:
    time.sleep(30)
    try:
        df = pd.read_csv("/home/sid/catkin_ws/src/sentinel_drone/yellow_box.csv")
    except:
        continue
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
    break