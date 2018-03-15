#!/usr/bin/env python3

# Python port of AbstractImageServer and SingleImageServer in sightedturtlesim

import numpy as np
import cv2


class SingleImageServer:
  # If the desired camera dimensions (with respect to the canvas dimensions)
  # exceeds DOWNSIZE_SCALE_RATIO times the dimensions of the desired image,
  # then the bounding box should be downsized prior to rotation, to ensure
  # that the downsized camera dimensions will be exactly DOWNSIZE_SCALE_RATIO
  # times the size of the desired image
  DOWNSIZE_SCALE_RATIO = 1.5

  def __init__(self, imageFilename, ppm=1.0):
    self._pixelsPerMeter = ppm
    self._imageFilename = imageFilename
    self._canvas = cv2.imread(imageFilename, flags=1) # flags=1 means return 3-channel color image
    self._height, self._width = self._canvas.shape[:2]

  """
  - headingDeg: 0=+x, 90=+y
  - output_shape: rows, columns
  """
  def getImage(self, eastingM, northingM, altitudeM, headingDeg, hfovDeg, aspectRatio, output_shape):
    # Convert inputs into pixel space
    xPx, yPx, zPx = eastingM*self._pixelsPerMeter, self._height-northingM*self._pixelsPerMeter, altitudeM*self._pixelsPerMeter
    thetaRad = headingDeg/180.0*np.pi
    camW = zPx*2. * np.tan(hfovDeg / 2.0 / 180.0 * np.pi)
    camH = camW / aspectRatio
    outputH, outputW = output_shape

    # Compute the bounding box width and height of the (rotated) camera frame
    # - order of corners: top-left (-x & -y), bottom-left, bottom-right, top-right
    camTransform = np.array(
      [[np.cos(thetaRad),  np.sin(thetaRad)],
      [-np.sin(thetaRad), np.cos(thetaRad)]])
    camCorners = np.array(
      [[-camW/2, -camW/2, +camW/2, +camW/2],
       [-camH/2, +camH/2, +camH/2, -camH/2]])
    camTransformedCorners = np.dot(camTransform, camCorners)
    camTXMax, camTXMin = np.max(camTransformedCorners[0,:])+xPx, np.min(camTransformedCorners[0,:])+xPx
    camTYMax, camTYMin = np.max(camTransformedCorners[1,:])+yPx, np.min(camTransformedCorners[1,:])+yPx

    # Decide to slightly over-sample the bounding box if rotation angle is not exact
    headingMod90 = (headingDeg % 90.)
    if headingMod90 > 45.: headingMod90 -= 90.
    if abs(headingMod90) > 5.0: # If headingDeg is not within +/- 5' away from 0', 90', 180', or 270'
      camTXMax += 1
      camTXMin -= 1
      camTYMax += 1
      camTYMin -= 1
        
    # Extract the sub-window corresponding to the bounding box
    camTXMax, camTXMin = int(round(camTXMax)), int(round(camTXMin))
    camTYMax, camTYMin = int(round(camTYMax)), int(round(camTYMin))
    if camTXMin >= 0 and camTXMax < self._width and camTYMin >= 0 and camTYMax < self._height:
      bbImage = self._canvas[camTYMin:camTYMax+1, camTXMin:camTXMax+1, :]
    else:
      bbImage = np.zeros(shape=(camTYMax-camTYMin+1, camTXMax-camTXMin+1, self._canvas.shape[2]), dtype=self._canvas.dtype)
      currCamTY = camTYMin
      bbY = 0
      while currCamTY <= camTYMax:
        currCamTYMod = currCamTY % self._height
        patchHeight = min(camTYMax-currCamTY+1, self._height-currCamTYMod)

        currCamTX = camTXMin
        bbX = 0
        while currCamTX <= camTXMax:
          currCamTXMod = currCamTX % self._width
          patchWidth = min(camTXMax-currCamTX+1, self._width - currCamTXMod)

          bbPatch = bbImage[bbY:bbY+patchHeight, bbX:bbX+patchWidth]
          np.copyto(bbPatch, self._canvas[currCamTYMod:currCamTYMod+patchHeight, currCamTXMod:currCamTXMod+patchWidth])

          currCamTX += patchWidth
          bbX += patchWidth

        currCamTY += patchHeight
        bbY += patchHeight

    # Decide to downsize image if necessary
    if camW > SingleImageServer.DOWNSIZE_SCALE_RATIO*outputW and camH > SingleImageServer.DOWNSIZE_SCALE_RATIO*outputH:
      downsizeFactor = max(SingleImageServer.DOWNSIZE_SCALE_RATIO*outputW/camW,
                           SingleImageServer.DOWNSIZE_SCALE_RATIO*outputH/camH)
      bbImage = cv2.resize(bbImage, dsize=None, fx=downsizeFactor, fy=downsizeFactor, interpolation=cv2.INTER_AREA)
      camW *= downsizeFactor
      camH *= downsizeFactor
      
    # Compute the width and height of the rotated bounding box
    # and adjust the centers of the transformation matrix
    bbImage_rows, bbImage_cols = bbImage.shape[:2]
    bbTransform = cv2.getRotationMatrix2D((bbImage_cols/2., bbImage_rows/2.), -headingDeg, 1.0)
    bbCorners = np.array(
      [[0, 0,            bbImage_cols, bbImage_cols],
       [0, bbImage_rows, bbImage_rows, 0           ],
       [1, 1,            1,            1           ]])
    bbTransformedCorners = np.dot(bbTransform, bbCorners)
    bbTWidth  = int(round(np.max(bbTransformedCorners[0,:]) - np.min(bbTransformedCorners[0,:])))
    bbTHeight = int(round(np.max(bbTransformedCorners[1,:]) - np.min(bbTransformedCorners[1,:])))

    bbTransform[0,2] += bbTWidth/2.0  - bbImage_cols/2.0
    bbTransform[1,2] += bbTHeight/2.0 - bbImage_rows/2.0
    bbRotatedImage = cv2.warpAffine(bbImage, bbTransform, (bbTWidth, bbTHeight), flags=cv2.INTER_NEAREST)
        
    bbRTopLeftX = max(int(bbRotatedImage.shape[1]/2. - camW/2.), 0)
    bbRTopLeftY = max(int(bbRotatedImage.shape[0]/2. - camH/2.), 0)
    bbRBottomRightX = bbRTopLeftX + max(int(camW), 1)
    if bbRBottomRightX > bbRotatedImage.shape[1]:
      bbRotatedImage = bbRotatedImage.shape[1]
    bbRBottomRightY = bbRTopLeftY + max(int(camH), 1)
    if bbRBottomRightY > bbRotatedImage.shape[0]:
      bbRBottomRightY = bbRotatedImage.shape[0]

    camImage = bbRotatedImage[bbRTopLeftY:bbRBottomRightY, bbRTopLeftX:bbRBottomRightX]
    if camImage.shape[:2] != output_shape:
      buffer = cv2.resize(camImage, dsize=(outputW, outputH), interpolation=cv2.INTER_LINEAR)
    else:
      buffer = camImage.copy() # In rare case camImage is a slice from underlying map

    # Visualize results
    if False:
      from matplotlib import pyplot as plt
      from matplotlib.path import Path
      import matplotlib.patches as patches

      XYs = camTransformedCorners.transpose()
      XYs[:,0] += xPx
      XYs[:,1] += yPx
      path = Path(XYs, [Path.MOVETO]+[Path.LINETO]*3)
      patch = patches.PathPatch(path, lw=2)
      
      plt.figure()
      ax = plt.subplot(221)
      ax.add_patch(patch)
      plt.xlabel('Eastings (m)')
      plt.ylabel('Northings (m)')
      plt.axis('equal')
      plt.axis([0, self._width, 0, self._height])
      ax.invert_yaxis()
      plt.title('patch')
      
      plt.subplot(222)
      plt.imshow(bbImage)
      plt.title('bbImage')

      plt.subplot(223)
      plt.imshow(bbRotatedImage)
      plt.title('bbRotatedImage')

      plt.subplot(224)
      plt.imshow(camImage)
      plt.title('camImage')
      
      plt.show()

      plt.figure()
      plt.imshow(camImage)
      plt.title('camImage')
      plt.show()

    return buffer

  """
  Returns cornersXY = [topLeftX, topLeftY, topRightX, topRightY,
                       bottomRightX, bottomRightY, bottomLeftX, bottomLeftY]
  """
  @staticmethod
  def toCornersXY(eastingM, northingM, altitudeM, headingDeg, hfovDeg, aspectRatio):
    # Convert inputs into pixel space
    xPx, yPx, zPx = eastingM*self._pixelsPerMeter, self._height-northingM*self._pixelsPerMeter, altitudeM*self._pixelsPerMeter
    thetaRad = headingDeg/180.0*np.pi
    camW = zPx*2. * np.tan(hfovDeg / 2.0 / 180.0 * np.pi)
    camH = camW / aspectRatio

    # Compute the bounding box width and height of the (rotated) camera frame
    # - order of corners: top-left (-x & -y), bottom-left, bottom-right, top-right
    camTransform = np.array(
      [[np.cos(thetaRad),  np.sin(thetaRad)],
      [-np.sin(thetaRad), np.cos(thetaRad)]])
    camCorners = np.array(
      [[-camW/2, -camW/2, +camW/2, +camW/2],
       [-camH/2, +camH/2, +camH/2, -camH/2]])
    camTransformedCorners = np.dot(camTransform, camCorners)
    camTransformedCorners[0,:] += xPx
    camTransformedCorners[1,:] += yPx
    cornersXY = np.ndarray.flatten(camTransformedCorners.transpose())
    return cornersXY
