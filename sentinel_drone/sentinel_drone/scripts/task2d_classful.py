import subprocess
from datetime import datetime

import cv2
from osgeo import gdal


class FeatDet_GeoRef:
    def __init__(self, base_img_path) -> None:
        # self.base_img_path = base_img_path
        self.base_img = cv2.imread(base_img_path)
        self.base_img = cv2.cvtColor(self.base_img, cv2.COLOR_BGR2GRAY)
        ds = gdal.Open(base_img_path)
        # GDAL affine transform parameters, According to gdal documentation xoff/yoff are image left corner, a/e are pixel wight/height and b/d is rotation and is zero if image is north up. 
        self.xoff, self.a, self.b, self.yoff, self.d, self.e = ds.GetGeoTransform()

    def featMatching(self, sml_img_path):
        sml_img = cv2.imread(sml_img_path)  
        sml_img = cv2.cvtColor(sml_img, cv2.COLOR_BGR2GRAY)
        start = datetime.now()
        sift = cv2.SIFT_create()
        keypoints_1, descriptors_1 = sift.detectAndCompute(self.base_img,None)
        keypoints_2, descriptors_2 = sift.detectAndCompute(sml_img,None)
        #feature matching
        bf = cv2.BFMatcher(cv2.NORM_L1, crossCheck=True)
        matches = bf.match(descriptors_1,descriptors_2)
        matches = sorted(matches, key = lambda x:x.distance)
        end = datetime.now()
        print("Time taken to match features = ",end - start,"\n")
        return keypoints_1, keypoints_2, matches

    def pixel2coord(self, x, y):
        """Returns global coordinates from pixel x, y coords"""
        xp = self.a * x + self.b * y + self.xoff
        yp = self.d * x + self.e * y + self.yoff
        return(xp, yp)
        
    def getCoordinatesQueryRun(self, sml_img_path):
        query = "gdal_translate "
        keypoints_1, keypoints_2, matches = self.featMatching(sml_img_path)
        for m in matches[:50]:
            x_base, y_base = (keypoints_1[m.queryIdx].pt)
            x_img, y_img = (keypoints_2[m.trainIdx].pt)
            x_new, y_new = self.pixel2coord(x_base, y_base)
            query += f"-gcp {x_img} {y_img} {x_new} {y_new} "
        query += f"-of GTiff {sml_img_path} map-with-gcps.tif"
        cmd1 = query.split(" ")
        for i in cmd1:
            if i == '':
                print(i)
                cmd1.remove(i)
        subprocess.run(cmd1)
        print("GDAL Translate done...\n")

    def geoReferenceQueryRun(self, img_with_gcp_path='map-with-gcps.tif'):
        t_srs = "+proj=longlat +ellps=WGS84"
        cmd2 = ["gdalwarp", "-overwrite", f"{img_with_gcp_path}","crs_updated.tif", "-t_srs", t_srs]
        subprocess.run(cmd2)
        print("GDAL Warp done...")


def main(sml_img_path, base_img_path='task2d.tif'):
    featobj = FeatDet_GeoRef(base_img_path)
    featobj.getCoordinatesQueryRun(sml_img_path)
    featobj.geoReferenceQueryRun()

if __name__=="__main__":
    main('test6.jpg',base_img_path='task2d.tif')