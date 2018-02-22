#!/usr/bin/env python3

# Python port of AbstractImageServer and SingleImageServer in sightedturtlesim

import numpy as np
import cv2


class AbstractImageServer:
  # If the desired camera dimensions (with respect to the canvas dimensions)
  # exceeds DOWNSIZE_SCALE_RATIO times the dimensions of the desired image,
  # then the bounding box should be downsized prior to rotation, to ensure
  # that the downsized camera dimensions will be exactly DOWNSIZE_SCALE_RATIO
  # times the size of the desired image
  DOWNSIZE_SCALE_RATIO = 1.5

  def __init__(self, w=0, h=0, ppm=1.0):
    self._width = w
    self._height = h
    self._pixelsPerMeter = ppm

  def __getImage(self, x, y, upDeg, camW, camH, output_shape):
    raise NotImplementedError()
  
  def _getImage(self, xPx, yPx, upDeg, zPx, hfovDeg, aspectRatio, output_shape):
    camW = zPx*2. * np.tan(hfovDeg / 2.0 / 180.0 * np.pi)
    camH = camW / aspectRatio
    return self.__getImage(xPx, yPx, upDeg, camW, camH, output_shape)

  def getImage(self, xM, yM, thetaRad, zM, hfovDeg, aspectRatio, output_shape):
    return self._getImage(xM * self._pixelsPerMeter, yM * self._pixelsPerMeter,
      -thetaRad/np.pi*180.0 + 90.0, zM * self._pixelsPerMeter, hfovDeg, aspectRatio, output_shape)

  def getMidX(self):
    return self._width / 2.0 / self._pixelsPerMeter
  def getMidY(self):
    return self._height / 2.0 / self._pixelsPerMeter

  """
  Returns cornersXY = [topLeftX, topLeftY, topRightX, topRightY,
                       bottomRightX, bottomRightY, bottomLeftX, bottomLeftY]
  # TODO: confirm headingDeg rotation by 90 degrees clockwise
  """
  @staticmethod
  def _toCornersXY(x, y, headingDeg, camW, camH):
    thetaRad = -headingDeg/180.0*np.pi
    camTransform = np.array(
      [[np.cos(thetaRad),  np.sin(thetaRad)],
       [-np.sin(thetaRad), np.cos(thetaRad)]])
    camCorners = np.array(
      [[-camW/2, +camW/2, +camW/2, -camW/2],
       [-camH/2, -camH/2, +camH/2, +camH/2]])
    camTransformedCorners = np.dot(camTransform, camCorners)
    camTC = np.ndarray.flatten(camTransformedCorners)
    cornersXY = [camTC[0] + x, camTC[4] + y, camTC[1] + x, camTC[5] + y,
                 camTC[2] + x, camTC[6] + y, camTC[3] + x, camTC[7] + y]
    return cornersXY

  @staticmethod
  def toCornersXY(x, y, headingDeg, z, hfovDeg, aspectRatio):
    camW = z*2. * np.tan(hfovDeg / 2.0 / 180.0 * np.pi)
    camH = camW / aspectRatio
    return AbstractImageServer._toCornersXY(x, y, headingDeg, camW, camH)


class SingleImageServer(AbstractImageServer):
  def __init__(self, imageFilename, ppm=1.0):
    self._imageFilename = imageFilename
    self._canvas = cv2.imread(imageFilename, flags=1) # flags=1 means return 3-channel color image
    super().__init__(self._canvas.shape[1], self._canvas.shape[0], ppm)

  # TODO: debug why polymorphism is failing
  def _getImage(self, xPx, yPx, upDeg, zPx, hfovDeg, aspectRatio, output_shape):
    camW = zPx*2. * np.tan(hfovDeg / 2.0 / 180.0 * np.pi)
    camH = camW / aspectRatio
    return self.__getImage(xPx, yPx, upDeg, camW, camH, output_shape)

  # TODO: debug why polymorphism is failing
  def getImage(self, xM, yM, thetaRad, zM, hfovDeg, aspectRatio, output_shape):
    return self._getImage(xM * self._pixelsPerMeter, yM * self._pixelsPerMeter,
      -thetaRad/np.pi*180.0 + 90.0, zM * self._pixelsPerMeter, hfovDeg, aspectRatio, output_shape)

  def __getImage(self, x, y, upDeg, camW, camH, output_shape):
    isWrapped = False
    buffer = np.zeros((output_shape[0], output_shape[1], self._canvas.shape[2]), dtype=self._canvas.dtype)

    # Process inputs
    thetaRad = -upDeg/180.0*np.pi

    # Compute the bounding box width and height of the (rotated) camera frame
    camTransform = np.array(
      [[np.cos(thetaRad),  np.sin(thetaRad)],
       [-np.sin(thetaRad), np.cos(thetaRad)]])
    camCorners = np.array(
      [[-camW/2, -camW/2, +camW/2, +camW/2],
       [-camH/2, +camH/2, -camH/2, +camH/2]])
    camTransformedCorners = np.dot(camTransform, camCorners)
    camTC = np.ndarray.flatten(camTransformedCorners)
    camTC[0] += x; camTC[1] += x; camTC[2] += x; camTC[3] += x;
    camTC[4] += y; camTC[5] += y; camTC[6] += y; camTC[7] += y;
    camTXMax = (max(max(camTC[0], camTC[1]), max(camTC[2], camTC[3])))
    camTXMin = (min(min(camTC[0], camTC[1]), min(camTC[2], camTC[3])))
    camTYMax = (max(max(camTC[4], camTC[5]), max(camTC[6], camTC[7])))
    camTYMin = (min(min(camTC[4], camTC[5]), min(camTC[6], camTC[7])))

    # Decide to slightly over-sample the bounding box if rotation angle is not exact
    upDegMod90 = upDeg - (upDeg//90)*90.0
    if upDegMod90 < -45.0: upDegMod90 += 90.0
    elif upDegMod90 > 45.0: upDegMod90 -= 45.0
    if abs(upDegMod90) > 5.0: # If upDeg is not within +/- 5' away from 0', 90', 180', or 270'
      camTXMax += 1.
      camTXMin -= 1.
      camTYMax += 1.
      camTYMin -= 1.

    # Extract the sub-window corresponding to the bounding box
    if round(camTXMin) >= 0 and round(camTXMax) < int(self._width) and round(camTYMin) >= 0 and round(camTYMax) < int(self._height):
      isWrapped = False
      bbImage = self._canvas[int(round(camTYMin)):int(round(camTYMax)) + 1, int(round(camTXMin)):int(round(camTXMax)) + 1, :]
    else:
      isWrapped = True
      bbImage = np.zeros(shape=(int(round(camTYMax - camTYMin + 1)), int(round(camTXMax - camTXMin + 1)), self._canvas.shape[2]), dtype=self._canvas.dtype)
      currCamTY = camTYMin
      bbY = 0
      while currCamTY <= camTYMax:
        currCamTYMod = int(round(currCamTY)) % int(self._height)
        if currCamTYMod < 0: currCamTYMod += self._height
        patchHeight = round(min(camTYMax - currCamTY + 1, self._height - currCamTYMod))

        currCamTX = camTXMin
        bbX = 0
        while currCamTX <= camTXMax:
          currCamTXMod = int(round(currCamTX)) % int(self._width)
          if currCamTXMod < 0: currCamTXMod += self._width
          patchWidth = round(min(camTXMax - currCamTX + 1, self._width - currCamTXMod))

          bbPatch = bbImage[int(bbY):int(bbY + patchHeight), int(bbX):int(bbX + patchWidth)]
          np.copyto(bbPatch, self._canvas[int(currCamTYMod):int(currCamTYMod + patchHeight), int(currCamTXMod):int(currCamTXMod + patchWidth)])

          currCamTX += patchWidth
          bbX += patchWidth

        currCamTY += patchHeight
        bbY += patchHeight

    # Decide to downsize image if necessary
    if camW > AbstractImageServer.DOWNSIZE_SCALE_RATIO*output_shape[1] and camH > AbstractImageServer.DOWNSIZE_SCALE_RATIO*output_shape[0]:
      downsizeFactor = max(AbstractImageServer.DOWNSIZE_SCALE_RATIO*output_shape[1]/camW,
                           AbstractImageServer.DOWNSIZE_SCALE_RATIO*output_shape[0]/camH)
      bbImage = cv2.resize(bbImage, dsize=None, fx=downsizeFactor, fy=downsizeFactor, interpolation=cv2.INTER_AREA)
      camW *= downsizeFactor
      camH *= downsizeFactor

    # Compute the width and height of the rotated bounding box
    #/ and adjust the centers of the transformation matrix
    bbImage_rows, bbImage_cols = bbImage.shape[:2]
    bbTransform = cv2.getRotationMatrix2D((bbImage_cols//2, bbImage_rows//2), upDeg, 1.0)
    bbCorners = np.array(
      [[0, 0,            bbImage_cols, bbImage_cols],
       [0, bbImage_rows, 0,            bbImage_rows],
       [1, 1,            1,            1]])
    bbTransformedCorners = np.dot(bbTransform, bbCorners)
    bbTC = np.ndarray.flatten(bbTransformedCorners)
    bbTWidth = round(max(max(bbTC[0], bbTC[1]), max(bbTC[2], bbTC[3])) -
        min(min(bbTC[0], bbTC[1]), min(bbTC[2], bbTC[3])))
    bbTHeight = round(max(max(bbTC[4], bbTC[5]), max(bbTC[6], bbTC[7])) -
        min(min(bbTC[4], bbTC[5]), min(bbTC[6], bbTC[7])))
    bbT = np.ndarray.flatten(bbTransform)

    bbT[2] = bbT[2] - bbImage_cols/2.0 + bbTWidth/2.0
    bbT[5] = bbT[5] - bbImage_rows/2.0 + bbTHeight/2.0

    # Rotate the bounding box and crop out the desired sub-image (via copy!)
    bbRotatedImage = cv2.warpAffine(bbImage, bbTransform, (int(bbTWidth), int(bbTHeight)), flags=cv2.INTER_NEAREST)
    bbRTopLeftX = max(int(bbRotatedImage.shape[1]/2.0 - camW/2), 0)
    bbRTopLeftY = max(int(bbRotatedImage.shape[0]/2.0 - camH/2), 0)
    bbRBottomRightX = bbRTopLeftX + max(int(camW), 1)
    if bbRBottomRightX > bbRotatedImage.shape[1]:
      bbRBottomRightX = bbRotatedImage.shape[1]
    bbRBottomRightY = bbRTopLeftY + max(int(camH), 1)
    if bbRBottomRightY > bbRotatedImage.shape[0]:
      bbRBottomRightY = bbRotatedImage.shape[0]

    camImage = bbRotatedImage[bbRTopLeftY:bbRBottomRightY, bbRTopLeftX:bbRBottomRightX]
    if camImage.shape[:2] != output_shape:
      buffer = cv2.resize(camImage, buffer.shape[:2], interpolation=cv2.INTER_LINEAR)
    else:
      buffer = camImage

    return buffer
