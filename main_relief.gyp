from selenium import webdriver
from selenium.webdriver.common.keys import Keys
import time
from math import *
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter

# change le chemin
PATH = 'C:\Program Files (x86)\chromedriver'
# driver = webdriver.Chrome(
# 'C:\\Users\\aubin\\AppData\\Local\\Packages\\PythonSoftwareFoundation.Python.3.\\LocalCache\\local-packages\\Python38\\Scripts\\chromedriver')
driver = webdriver.Chrome(PATH)
driver.get('https://www.freemaptools.com/elevation-finder.htm')

time.sleep(0.5)
cookies = driver.find_element_by_xpath(
    "/html/body/div[4]/div[2]/div[1]/div[2]/div[2]/button[1]")
cookies.click()

latentrée = 47.21400
longentrée = -1.56700
latsortie = 47.25000
longsortie = -1.52000

longueur = (longsortie-longentrée)*111
largeur = (latsortie-latentrée)*111*cos((longsortie-longentrée)*pi/360)

paslong = abs(longsortie-longentrée)/longueur
paslarg = abs(latsortie-latentrée)/largeur

longe = 0
lat = 0
LONGE = longentrée
LAT = latentrée
Listalt = []
Listlonge = []
Listlat = []
Listfinale = []

for k in range(int(longueur)+1):
    longe += 1
    LONGE = longentrée + k*paslong
    for j in range(int(largeur)+1):
        lat += 1
        LAT = latentrée + j*paslarg

        elem = driver.find_element_by_id("locationSearchTextBox")
        elem.clear()
        elem.click()
        elem.send_keys(str(LAT), ',', str(LONGE))
        elem.send_keys(Keys.ENTER)

        time.sleep(0.2)
        text_element = driver.find_element_by_xpath(
            "/html/body/div[2]/div[2]/div[4]")

        if text_element.text == "Please Wait...":
            time.sleep(1.5)

        text_element = driver.find_element_by_xpath(
            "/html/body/div[2]/div[2]/div[4]")
        text = text_element.text

        i = 0
        altitude = ''
        while text[i] != 'm':
            altitude += text[i]
            i += 1
        Listalt.append(float(altitude))
        Listlonge.append(longe)
        Listlat.append(lat)
        Listfinale.append([longe, lat, float(altitude)])

# driver.close()

print(Listfinale)
