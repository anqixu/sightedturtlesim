#!/usr/bin/env python
from math import *

M_PI = pi
EARTH_RADIUS_M = 6367500.0
DEG_TO_RAD = M_PI/180.0
RAD_TO_DEG = 180.0/M_PI
INVALID_DEGREE_VALUE = 361.0
BTC40_WIDTH = 752
BTC40_HEIGHT = 582
NAN = float('nan')

def EarthMeter2Deg(meter):
  return meter / EARTH_RADIUS_M * RAD_TO_DEG


def EarthDeg2Meter(deg):
  return deg * DEG_TO_RAD * EARTH_RADIUS_M


def Bearing2GCS(startLatDeg, startLonDeg, distMeter, absHeadingDeg):
  if distMeter < 0:
    distMeter = -distMeter
    absHeadingDeg = absHeadingDeg + 180.0

  DOverR = distMeter / EARTH_RADIUS_M
  startLatRad = startLatDeg * DEG_TO_RAD
  absHeadingRad = absHeadingDeg * DEG_TO_RAD
  destLatRad = asin(sin(startLatRad)*cos(DOverR) + \
    cos(startLatRad)*sin(DOverR)*cos(absHeadingRad))
  destLonDeg = startLonDeg + \
    atan2(sin(absHeadingRad)*sin(DOverR)*cos(startLatRad), \
    cos(DOverR) - sin(startLatRad)*sin(destLatRad)) * RAD_TO_DEG
  destLatDeg = destLatRad * RAD_TO_DEG

  return (destLatDeg, destLonDeg)


def Meter2GCS(startLatDeg, startLonDeg, xMeter, yMeter):
  return Bearing2GCS(startLatDeg, startLonDeg, \
      sqrt(xMeter*xMeter+yMeter*yMeter), atan2(xMeter, -yMeter)*RAD_TO_DEG)


def GCS2Bearing(startLatDeg, startLonDeg, currLatDeg, currLonDeg):
  startLatRad = startLatDeg * DEG_TO_RAD
  currLatRad = currLatDeg * DEG_TO_RAD
  deltaLonRad = (currLonDeg - startLonDeg) * DEG_TO_RAD
  sinDeltaLatHalf = sin((currLatRad - startLatRad)/2)
  sinDeltaLonHalf = sin(deltaLonRad/2)
  a = sinDeltaLatHalf*sinDeltaLatHalf + \
      cos(startLatRad)*cos(currLatRad)*sinDeltaLonHalf*sinDeltaLonHalf

  distMeter = 2*atan2(sqrt(a), sqrt(1-a)) * EARTH_RADIUS_M
  absHeadingDeg = atan2(sin(deltaLonRad)*cos(currLatRad), \
      cos(startLatRad)*sin(currLatRad) - \
      sin(startLatRad)*cos(currLatRad)*cos(deltaLonRad)) * RAD_TO_DEG

  return (distMeter, absHeadingDeg)


def GCS2Meter(startLatDeg, startLonDeg, currLatDeg, currLonDeg):
  bearing = GCS2Bearing(startLatDeg, startLonDeg, currLatDeg, currLonDeg)
  distMeter = bearing[0]
  absHeadingRad = bearing[1] * DEG_TO_RAD
  return (distMeter * sin(absHeadingRad), -distMeter * cos(absHeadingRad))


def getCoordTransform(orientationDeg, pitchDeg, rollDeg, azimuthDeg, elevationDeg):
  result = []

  # NOTE: the following simplifications were obtained using MATLAB's symbolic math library
  co = cos(orientationDeg * DEG_TO_RAD)
  so = sin(orientationDeg * DEG_TO_RAD)
  cp = cos(pitchDeg * DEG_TO_RAD)
  sp = sin(pitchDeg * DEG_TO_RAD)
  cr = cos(rollDeg * DEG_TO_RAD)
  sr = sin(rollDeg * DEG_TO_RAD)
  ca = cos(azimuthDeg * DEG_TO_RAD)
  sa = sin(azimuthDeg * DEG_TO_RAD)
  ce = cos(elevationDeg * DEG_TO_RAD)
  se = sin(elevationDeg * DEG_TO_RAD)

  result.append(ca*(co*cr + so*sp*sr) - cp*sa*so)
  result.append(ce*(sa*(co*cr + so*sp*sr) + ca*cp*so) - se*(co*sr - cr*so*sp))
  result.append(ce*(co*sr - cr*so*sp) + se*(sa*(co*cr + so*sp*sr) + ca*cp*so))
  result.append(-ca*(cr*so - co*sp*sr) - co*cp*sa)
  result.append(se*(so*sr + co*cr*sp) - ce*(sa*(cr*so - co*sp*sr) - ca*co*cp))
  result.append(-ce*(so*sr + co*cr*sp) - se*(sa*(cr*so - co*sp*sr) - ca*co*cp))
  result.append(-sa*sp - ca*cp*sr)
  result.append(ce*(ca*sp - cp*sa*sr) - cp*cr*se)
  result.append(se*(ca*sp - cp*sa*sr) + ce*cp*cr)

  return result


def getGroundProjection(T, iPx, jPx, wPx, hPx, horizFOVDeg, altitudeM, aspectRatio = 0):
  # NOTE: the following simplifications were obtained using MATLAB's symbolic math library
  half_width = tan(horizFOVDeg/2/180*M_PI)

  if aspectRatio <= 0:
    aspectRatio = wPx/hPx

  half_height = half_width/aspectRatio
  iM = (2*iPx/wPx - 1)*half_width
  jM = (1 - 2*jPx/hPx)*half_height
  denom = T[7] + T[6]*iM + T[8]*jM

  if denom >= 0: # the Z coordinate of the transformed line points upwards from the UAV
    return (NAN, NAN)

  xM = -(altitudeM*(T[1] + T[0]*iM + T[2]*jM))/denom
  yM = -(altitudeM*(T[4] + T[3]*iM + T[5]*jM))/denom
  return (xM, yM)

