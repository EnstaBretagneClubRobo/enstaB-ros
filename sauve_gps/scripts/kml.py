import sys
kmlBody = ('')


def addPoint(i,Lat,Long):
    kml = (
        '<Placemark>\n'
        '<name>OPI</name>\n'
        '<description>\n'
        '%d-opi wait'
        '</description>\n'
        '<Point>\n'
        '<coordinates>%f,%f,0</coordinates>\n'
        '</Point>\n'
        '</Placemark>\n'
        ) %(i,Lat,Long)
    open('/home/nuc1/data/opi.kml','a').write(kml)
 
def addGPS(num,Lat,Long):
    kml = (
        '<Placemark>\n'
        '<name>GPS-%d</name>\n'
        '<description>GPS-Point\n'
        '</description>\n'
        '<Point>\n'
        '<coordinates>%f, %f,0</coordinates>\n'#\'%0.2f\'\'N/ %d %d\'%0.2f\'\'E</coordinates>\n'
        '</Point>\n'
        '</Placemark>\n'
        ) %(num,Lat,Long)#[0],#Lat[1],Lat[2],Long[0],Long[1],Long[2])
    open('/home/nuc1/data/gps.kml','a').write(kml)
#"morceaux" du fichier KML 
 
 

def openFicGPS():
    kmlHeader = ('<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n'
             '<kml xmlns=\"http://earth.google.com/kml/2.2\">\n'
             '<Document>\n')
    open('/home/nuc1/data/gps.kml','w').write(kmlHeader)

def openFicOPI():
    kmlHeader = ('<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n'
             '<kml xmlns=\"http://earth.google.com/kml/2.2\">\n'
             '<Document>\n')
    open('/home/nuc1/data/opi.kml','w').write(kmlHeader)

    
def close():
    kmlFooter = ('</Document>\n'
             '</kml>\n')
    open('/home/nuc1/data/opi.kml','a').write(kmlFooter)


def closeGPS():
    kmlFooter = ('</Document>\n'
             '</kml>\n')
    open('/home/nuc1/data/gps.kml','a').write(kmlFooter)


if __name__ == '__main__':
   if sys.argv[1]=='w':
         addPoint(argv[2],argv[3],argv[4],argv[5],argv[6])
