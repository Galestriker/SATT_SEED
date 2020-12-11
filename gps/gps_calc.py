import numpy as np

r  = 6378.137e3#地球半径
def azimuth(lat1, lon1, lat2, lon2):#lat:経度lon:緯度 (lat1,lon1)から他(lat2,lon2)の目標角を返す
    # Radian角に修正
    _lat1, _lon1, _lat2, _lon2 = lat1*np.pi/180, lon1*np.pi/180, lat2*np.pi/180, lon2*np.pi/180
    Δx = _lat2 - _lat1
    _y = np.sin(Δx)
    _x = np.cos(_lon1) * np.tan(_lon2) - np.sin(_lon1) * np.cos(Δx)

    psi = np.arctan2(_y, _x) * 180 / np.pi
    if psi < 0:
        return 360 + np.arctan2(_y, _x) * 180 / np.pi
    else:
        return np.arctan2(_y, _x) * 180 / np.pi

def distance(lat1, lon1, lat2, lon2):#2点間距離をmで返す．
    _lat1, _lon1, _lat2, _lon2 = lat1*np.pi/180, lon1*np.pi/180, lat2*np.pi/180, lon2*np.pi/180
    Δx = _lat2 - _lat1
    val = np.sin(_lon1) * np.sin(_lon2) + np.cos(_lon1) * np.cos(_lon2) * np.cos(Δx)
    return r * np.arccos(val)

if __name__ == '__main__':
    x1 = 139.988909
    y1 = 35.685828
    x2 = 139.990339
    y2 = 35.685879
    angle = azimuth(x1, y1, x2, y2)
    dis   = distance(x1, y1, x2, y2) / 1e3 # kmに変換
    print("方位角 : {0:.3f} 度".format(angle))
    print("距離 : {0:.3f} km".format(dis))