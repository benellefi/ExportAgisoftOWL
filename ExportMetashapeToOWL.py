##################################################
# This script enables to export an Agisoft Metashape project to an OWL ABox modeled with Arpenteur ontology
##################################################
## {License_info}
##################################################
## Author: {Pierre DRAP & Mohamed BEN ELLEFI}
## Copyright: Copyright {2019}, {LIS/AMU/CNRS}
## License: {http://creativecommons.org/licenses/by/4.0/}
## Email: {Pierre.DRAP@univ-amu.fr}
##################################################


import PhotoScan
from math import *
from owlready2 import *
# This script is tested with Owlready2 0.14



# ==========================================================
# //////////////////////////////////////////////////////////
# Storage class to efficiently handle projections
class PointCoordsAndCamId:
    def __init__(self, pointCoords, id):
        self.myCoords = pointCoords
        self.myId = id


#
#
#
exportedSensorList = list()

#
#
#
exportedCameraList = list()


def basicInstanceGenerator(onto):
    """

    :type onto: object
    """
    classes = list(onto.classes())  # Get all classes from the ontology
    print("-----------------------------------")
    print("class list:")
    print("-----------------------------------")
    print(classes)
    prop = list(onto.properties())  # Get all properties from the ontology
    print("-----------------------------------")
    print("properties list:")
    print("-----------------------------------")
    print(prop)
    indi = list(onto.individuals())  # Get all individuals from the ontology
    print("-----------------------------------")
    print("individuals list:")
    print("-----------------------------------")
    print(indi)
    print("class CameraManager")
    print(onto.CameraManager.iri)
    cameraMan1 = onto.CameraManager()
    cameraMan2 = onto.CameraManager("toto2")
    print("individuals camera Manager: ")
    print(onto.toto2)
    print("individuals list:")
    insta = list(onto.individuals())
    print(insta)
  #  print("onto.search_one(is_a = onto.CameraManager)")
  #  print(onto.search_one(is_a = onto.CameraManager))
  #  print("onto.search_one(label = toto1)")
  #  print(onto.search_one(label = "toto1"))
  #  for prop in onto.CameraManager().get_properties():
  #      for value in prop[onto.CameraManager()]:
  #          print(".%s == %s" % (prop.python_name, value))
    print("saving")
    onto.save(file="C:/Users/benellefi/PycharmProjects/onto/totototo.owl", format="rdfxml")
    print("saved")
    pass


def rotY(angle):
    sinAngle = sin(angle)
    cosAngle = cos(angle)

    mat = PhotoScan.Matrix(
        [[cosAngle, 0., sinAngle, 0.], [0., 1., 0., 0.], [-sinAngle, 0., cosAngle, 0], [0., 0., 0., 1.]])
    # print("matY " + str(mat))
    return mat


def rotZ(angle):
    sinAngle = sin(angle)
    cosAngle = cos(angle)

    mat = PhotoScan.Matrix(
        [[cosAngle, -sinAngle, 0., 0.], [sinAngle, cosAngle, 0., 0.], [0., 0., 1., 0], [0., 0., 0., 1.]])
    # print("matZ " + str(mat))
    return mat


# ==========================================================
# //////////////////////////////////////////////////////////
# Creates a dictionnary to handle efficiently projections
def createProjectionDict(numberCameras, chunk):
    points3D = chunk.point_cloud.points

    projListByTrackIdDict = dict()

    for i in range(0, numberCameras):
        # Retrieve the pointCloudProjection from the i_th PointCloudCamera
        camera = chunk.cameras[i]

        # If that photo contains 2D matching points informations
        if camera in chunk.point_cloud.projections:
            # Get the list of 2D points
            projByCam = chunk.point_cloud.projections[camera]

            # For each 2D point related to the i_th photo
            for j in range(0, len(projByCam)):
                trackId = projByCam[j].track_id

                if not trackId in projListByTrackIdDict:
                    projListByTrackIdDict[trackId] = list()

                projListByTrackIdDict[trackId].append(PointCoordsAndCamId(projByCam[j].coord, i))

    return projListByTrackIdDict


# ==========================================================
# //////////////////////////////////////////////////////////
# Checks whether the given sensor was already exported
#
# exportedSensorList : List of already exported sensors
# sensor : the sensor to check
#
# return : True if the sensor was exported before, false otherwise
#
#  note: sensor is camera in Arpenteur terminology
#
def isSensorExported(exportedSensorList, sensor):
    for i, name in enumerate(exportedSensorList):
        if name == sensor.label:
            return True
    return False


# ==========================================================
# //////////////////////////////////////////////////////////
# Checks whether the given camera was already exported
#
# exportedCameraList : List of already exported camera
# camera : the camera to check
#
# return : True if the camera was exported before, false otherwise
#
#  note: Camera is photograph in Arpenteur terminology
#
def isCameraExported(exportedCameraList, camera):
    for i, name in enumerate(exportedCameraList):
        if name == camera.label:
            return True
    return False


# ==========================================================
# //////////////////////////////////////////////////////////

def calibrationParameters(sensor):


    # Retrieve photoscan parameters
    f = sensor.calibration.f
    # fy = sensor.calibration.fy
    cx = sensor.calibration.cx
    cy = sensor.calibration.cy
    focalLength = sensor.focal_length
    width = sensor.calibration.width
    height = sensor.calibration.height
    k1 = sensor.calibration.k1
    k2 = sensor.calibration.k2
    k3 = sensor.calibration.k3
    k4 = sensor.calibration.k4  # used in Arpenteur models :)
    p1 = sensor.calibration.p1
    p2 = sensor.calibration.p2
    # skew = sensor.calibration.skew
    pixelSize = sensor.pixel_size

    # new
    if (sensor.pixel_size is not None and sensor.pixel_size.x != 0):
        pixelSize = sensor.pixel_size

        # Arpenteur needs square pixels, so adjust the size in case
        if (pixelSize.x != pixelSize.y):
            pixelSize.x = (pixelSize.x + pixelSize.y) / 2
            pixelSize.y = pixelSize.x

        # Recompute the focal length
        focalLength = pixelSize.x * f
        sensor.focal_length = focalLength
    # end new
    elif focalLength is not None:
        pixelSize = PhotoScan.Vector((focalLength / f, focalLength / f))
    else:  # No focal nor pixel size is provided, let's prompt the user for the values
        paramType = PhotoScan.app.getInt(
            "Write 1 to define pixel size or 2 to define camera width for camera \"" + sensor.label + "\" (Pixel size / focal length should have been defined before alignment for more accurate results).")

        if paramType == 1:
            fPixelSize = PhotoScan.app.getFloat("Pixel size (mm) ?")
            sensor.pixel_size = pixelSize = PhotoScan.Vector((fPixelSize, fPixelSize))

        elif paramType == 2:
            frameWidth = PhotoScan.app.getFloat("Frame width (mm) ?")
            # Assume square pixels
            sensor.pixel_size = pixelSize = PhotoScan.Vector((frameWidth / sensor.width, frameWidth / sensor.width))
        else:
            raise ValueError('Pixel size or focal length must be provided.')

        print("   camera pixelSize: " + pixelSize)
        # Recompute the focal length
        focalLength = pixelSize.x * f
        sensor.focal_length = focalLength

    # print("focal " + str(focalLength))

    # Fiducial markers
    mmSizeX = pixelSize.x * width
    mmSizeY = pixelSize.y * height
    maxX = mmSizeX / 2
    minX = - maxX
    maxY = mmSizeY / 2
    minY = - maxY

    # ppx / ppy (offset wrt image center in PS 1.2)
    ppx = cx * pixelSize.x
    ppy = -cy * pixelSize.y
    # ppx = (cx - width / 2) * pixelSize.x
    # ppy = (height / 2 - cy) * pixelSize.y

    # Distortion
    k1 = k1 / (focalLength * focalLength)
    k2 = k2 / pow(focalLength, 4)
    k3 = k3 / pow(focalLength, 6)
    k4 = k4 / pow(focalLength, 8)  # used in Arpenteur Model

    invK1 = -k1
    invK2 = 3 * k1 * k1 - k2
    invK3 = -12 * pow(k1, 3) + 8 * k1 * k2 - k3
    invK4 = 55 * pow(k1, 4) - 55 * pow(k1, 2) * k2 + 5 * pow(k2, 2) + 10 * k1 * k3 - k4
    # invK4 = 55 * pow(k1, 4) + 9 * k1 * k3 - 51 * k1 * k1 * k2 + 5 * k2 * k2 - k4
      # To be checked
    P1 = -p2 / pow(focalLength, 2)  # sign ? fx or f (mm) ?
    P2 = -p1 / pow(focalLength, 2)  # sign ? fx or f (mm) ?
    # P1 = -p2 / pow(f, 2) #sign ? fx or f (mm) ?
    # P2 = -p1 / pow(f, 2) #sign ? fx or f (mm) ?
    distortion = str(invK1) + " " + str(invK2) + " " + str(invK3) + " " + str(invK4) + " " + str(P1) + " " + str(P2)
    # distortion = str(invK1) + " " + str(invK2) + " "  + str(invK3) + " " + " " + str(P1) + " " + str(P2)
    print("		 distortion: " + distortion)
    print("		 focal	 : " + str(focalLength))
    print("		 ppx ppy   : " + str(ppx) + "	" + str(ppy))
    cameraExportOWL([sensor.key, sensor.label, focalLength, ppx, ppy], [invK1, invK2, invK3, invK4, P1, P2])

def cameraExportOWL(sensor, distortion):

    cameraOWL = onto.Camera("Camera_"+str(sensor[0]))
    cameraOWL.hasName = [sensor[1]]
    cameraOWL.hasFocalLength = [sensor[2]]
    cameraOWL.hasPPX = [sensor[3]]
    cameraOWL.hasPPY = [sensor[4]]
    radialDecenteringDistortion = onto.RadialDecenteringDistortion("RadialDecenteringDistortionCamera_"+str(sensor[0]))
    cameraOWL.hasDistortion = [radialDecenteringDistortion]
    radialDecenteringDistortion.hasCoef_K1 = [distortion[0]]
    radialDecenteringDistortion.hasCoef_K2 = [distortion[1]]
    radialDecenteringDistortion.hasCoef_K3 = [distortion[2]]
    radialDecenteringDistortion.hasCoef_K4 = [distortion[3]]
    radialDecenteringDistortion.hasCoef_P1 = [distortion[4]]
    radialDecenteringDistortion.hasCoef_P2 = [distortion[5]]

    cameraManagerOWL.hasASetOfCamera.append(cameraOWL)

def externalParameters(chunk, photo):
    toChunkCoordTransform = photo.transform
    # if no orientation was computation, go on
    if toChunkCoordTransform is not None:
        chunkToWorldTransform = chunk.transform.matrix
        # Transform matrix to world coords (will be inverted in Arpenteur)
        worldCoordTransform = chunkToWorldTransform * toChunkCoordTransform
        tmp = worldCoordTransform * rotY(-pi)
        tmp2 = tmp * rotZ(-pi)
        worldCoordTransform = tmp2
        # Get the matrix of external parameters :
        # inverse of the matrix describing photo location in the chunk coord system * inv of matrix describing the chunk location in
        # the world coordinate system
        # worldCoordTransform = chunkCoordTransform.inv() * matInv
        r00 = worldCoordTransform[0, 0]
        r01 = worldCoordTransform[0, 1]
        r02 = worldCoordTransform[0, 2]
        r10 = worldCoordTransform[1, 0]
        r11 = worldCoordTransform[1, 1]
        r12 = worldCoordTransform[1, 2]
        r20 = worldCoordTransform[2, 0]
        r21 = worldCoordTransform[2, 1]
        r22 = worldCoordTransform[2, 2]
        tx = worldCoordTransform[0, 3]
        ty = worldCoordTransform[1, 3]
        tz = worldCoordTransform[2, 3]
        print("		 matrix: ")
        print("			   " + str(r00) + "   " + str(r01) + "   " + str(r02))
        print("			   " + str(r10) + "   " + str(r11) + "   " + str(r12))
        print("			   " + str(r20) + "   " + str(r21) + "   " + str(r22))
        print("		 translation: ")
        print("			   " + str(tx) + "   " + str(ty) + "   " + str(tz))
        print("		 file: " + photo.photo.path)
        print(" ")
        exportPhotographOWL(photo, worldCoordTransform)

def exportPhotographOWL(photo, worldCoordTransform):
    photographOWL = onto.Photograph("Phtograph_" + str(photo.key))
    photographOWL.hasName = [str(photo.label)]
    photographOWL.hasFullFileName = [str(photo.photo.path)]
    idcam = arpIRI+"Camera_"+str(photo.sensor.key)
    photographOWL.hasCamera = [IRIS[idcam]]

    transformation3D = onto.Transformation3D("Transformation3DPhtograph" + str(photo.key))
    photographOWL.hasTransformation3D = [transformation3D]

    Matrix3D = onto.RotationMatrix("Matrix3DPhtograph_" + str(photo.key))
    transformation3D.hasRotationMatrix = [Matrix3D]
    Matrix3D.has_m10 = [worldCoordTransform[0, 0]]
    Matrix3D.has_m11 = [worldCoordTransform[0, 1]]
    Matrix3D.has_m12 = [worldCoordTransform[0, 2]]
    Matrix3D.has_m20 = [worldCoordTransform[1, 0]]
    Matrix3D.has_m21 = [worldCoordTransform[1, 1]]
    Matrix3D.has_m22 = [worldCoordTransform[1, 2]]


    translation3D = onto.IPoint3D("Translation3DPhtograph_" + str(photo.key))
    transformation3D.hasTranslation = [translation3D]
    translation3D.hasX = [worldCoordTransform[0, 3]]
    translation3D.hasY = [worldCoordTransform[1, 3]]
    translation3D.hasZ = [worldCoordTransform[2, 3]]

    photoManagerOWL.haveASetOfPhotograph.append(photographOWL)


def exportSensor(chunk):
    print("")
    print("  -- exporting Arpenteur cameras")
    print("")
    numberCameras = len(chunk.cameras)
    for i in range(0, numberCameras):
        sensor = chunk.cameras[i].sensor
        if isSensorExported(exportedSensorList, sensor):
            continue
        else:
            print("	Exporting Arpenteur camera " + sensor.label)
            exportedSensorList.append(sensor.label)
            calibrationParameters(sensor)

    print("")
    print("  " + str(len(exportedSensorList)) + " exported Arpenteur cameras")
    print("")



def exportCamera(chunk):
    print("")
    print("  -- exporting Arpenteur photographs")
    print("")
    numberCameras = len(chunk.cameras)
    for i in range(0, numberCameras):
        camera = chunk.cameras[i]
        if isCameraExported(exportedCameraList, camera):
            continue
        else:
            print("	Exporting Arpenteur photograph " + camera.label)
            exportedCameraList.append(camera.label)
            externalParameters(chunk, camera)

    print("")
    print("  " + str(len(exportedCameraList)) + " exported Arpenteur photographs")
    print("")




def exportPts(numberCameras, chunk, dict2D):
    print("")
    print("  -- exporting Arpenteur pts")
    print("")
    points3D = chunk.point_cloud.points

    for point3D in points3D:
        id = point3D.track_id
        vec = point3D.coord.copy();
        vec.size = 3
        worldPoint3D = chunk.transform.matrix.mulp(vec)
        print("	pt3D " + str(id) + "  x " + str(worldPoint3D.x) + "  y " + str(worldPoint3D.y) + " z " + str(
            worldPoint3D.z))
        imagePointManagerOWL = exportiPoint3DToOWL(id, worldPoint3D)
        projListById = dict2D.get(id, None)
        if (projListById is not None):
            for j in range(0, len(projListById)):
                photo = chunk.cameras[projListById[j].myId]
                toChunkCoordTransform = photo.transform
                # if no orientation was computated, go on
                if toChunkCoordTransform is not None:
                    point2D = projListById[j].myCoords
                    print("	  proj on photo " + photo.label + "	 x " + str(point2D[0]) + "  y " + str(point2D[1]))
                    exportiPoint2DToOWL(id, projListById[j].myId, imagePointManagerOWL, photo.key, point2D)



def exportiPoint3DToOWL( id, worldPoint3D):
    iPoint3DOWL = onto.IPoint3D("IPoint3D_" + str(id))
    iPoint3DOWL.hasX = [worldPoint3D.x]
    iPoint3DOWL.hasY = [worldPoint3D.y]
    iPoint3DOWL.hasZ = [worldPoint3D.z]

    imagePointManagerOWL = onto.ImagePointManager("ImagePointManagerIPoint3D_" + str(id))
    iPoint3DOWL.hasImagePointManager = [imagePointManagerOWL]
    measuredPointManagerOWL.hasASetOf3DPointWithObs.append(iPoint3DOWL)

    return imagePointManagerOWL

def exportiPoint2DToOWL(point3D_id, point2D_id, imagePointManagerOWL, photokey, point2D):
    iPoint2DOWL = onto.ImagePoint("ImagePoint_IPoint3D_" + str(point3D_id)+"_Phtograph_"+str(point2D_id))
    iPoint2DOWL.hasX = [point2D[0]]
    iPoint2DOWL.hasY = [point2D[1]]
    iriPhotograph = arpIRI + "Phtograph_" + str(photokey)
    iPoint2DOWL.hasPhotograph = [IRIS[iriPhotograph]]
    imagePointManagerOWL.hasASetOfObservation.append(iPoint2DOWL)


# ============================================================
# ////////////////////////////////////////////////////////////
# Main function
def main():
    global gFile
    global onto
    global arpIRI
    global measuredPointManagerOWL
    global photoManagerOWL
    global cameraManagerOWL
    print("-----------------------------------------------------------------")
    print("---------- exporting photoscan model in arpenteur ABOX  ---------")
    print("-----------------------------------------------------------------")
    doc = PhotoScan.app.document
    chunk = doc.chunk
    numberCameras = len(chunk.cameras)
    # Opens save-to dialog box
    path = "/"
    # path = PhotoScan.app.getSaveFileName("Specify the destination file for the exported OWL ABOX :")
    print(path)
    if path == "":
        return

    outFile = path  # + "/testExportArpenteur.owl" #to change

    if outFile[-4:] != ".owl":
        outFile = outFile + ".owl"

    print("Photoscan model will exported in: " + outFile)
    # onto = get_ontology("http://www.lesfleursdunormal.fr/static/_downloads/pizza_onto.owl")

    #onto = None
    #onto_path.append("/onto/")
    #onto = get_ontology("http://www.arpenteur.org/ontology/Arpenteur.owl")
    onto = get_ontology("http://www.arpenteur.org/ontology/ArpenteurV14.owl")
    onto.load()
    arpIRI = "http://www.arpenteur.org/ontology/Arpenteur.owl#"
    #basicInstanceGenerator(onto)
    print("  -- Arpenteur loaded")

    onto.Model()

    modelOWL = onto.Model("Model" + str(chunk.key))
    modelOWL.hasName = ["Model_" + str(chunk.label)]
    measuredPointManagerOWL = onto.MeasuredPointManager("MeasuredPointManager" + str(chunk.key))
    measuredPointManagerOWL.hasName = ["MeasuredPointManager_" + str(chunk.label)]
    modelOWL.hasMeasuredPointManager = [measuredPointManagerOWL]
    cameraManagerOWL = onto.CameraManager("CameraManager" + str(chunk.key))
    cameraManagerOWL.hasName = ["CameraManager_" + str(chunk.label)]
    modelOWL.hasCameraManager = [cameraManagerOWL]
    photoManagerOWL = onto.PhotoManager("PhotoManager" + str(chunk.key))
    photoManagerOWL.hasName = ["PhotoManager_" + str(chunk.label)]
    modelOWL.hasPhotoManager = [photoManagerOWL]

    print("Begin projection dictionary")
    projById = createProjectionDict(numberCameras, chunk)
    # exporting Arpenteur cameras
    exportSensor(chunk)
    # exporting Arpenteur photographs
    exportCamera(chunk)
    # exporting Arpenteur Pts 3D & 2D
    exportPts(numberCameras, chunk, projById)

    print("  -- now saving ABOX")
    # onto.save()
  #  if not os.path.exists(os.path.realpath('..')+"/onto/"):
  #     os.makedirs(os.path.realpath('..')+"/onto/")
   #The exported ontology will be in the same directory as your Photoscan project
    fileName = PhotoScan.app.getSaveFileName("Specify ABOX filename for saving:")
    if not fileName:
        print("Script aborted")

    if fileName[-4:].lower() != ".owl":
        fileName += ".owl"

    onto.save(file=fileName, format="rdfxml")
    print("-----------------------------------------------------------------")
    print("---------------- end of exporting photoscan in OWL --------------")
    print("-----------------------------------------------------------------")


# =============================================
# /////////////////////////////////////////////
# Start
if __name__ == "__main__":
    main()
