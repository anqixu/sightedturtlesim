#!/usr/bin/env python

import argparse
import cv2
import numpy as np
import os
import pandas as pd
import sys
import tempfile
import tqdm
import shutil
from ImageServer import SingleImageServer


class PoseXYZ:
  def __init__(self, x=0, y=0, z=0, theta=0):
    self.x = x
    self.y = y
    self.z = z
    self.theta = theta


class PoseSampler(object):
  @staticmethod
  def parse_args():
    parser = argparse.ArgumentParser(description='ImageServer Dataset Generator')
    parser.add_argument('--output', type=str, default='dataset.zip',
                        help='PNG-zip output file storing images and poses [dataset.zip]')
    parser.add_argument('--map', type=str, default='',
                        help='Map to load []')
    parser.add_argument('--ppm', type=float, default=1.0,
                        help='Map pixel-per-meter ratio [1.0]')
    parser.add_argument('--hfov', type=float, default=100,
                        help='Camera horizontal field of view (deg) [100]')
    parser.add_argument('--width', type=int, default=224,
                        help='Image width (px) [224]')
    parser.add_argument('--height', type=int, default=224,
                        help='Image height (px) [224]')
    parser.add_argument('--nxyz', type=int, default=100,
                        help='Number of Pose.x/Pose.y(/Pose.z)(/Pose.theta for uniform) [100]')
    parser.add_argument('--z', type=float, default=100.0,
                        help='Fixed/min. Pose.z (m) [100.0]')
    parser.add_argument('--dz', type=float, default=0.0,
                        help='Range of Pose.z (m) [0.0]')
    parser.add_argument('--t', type=float, default=0.0,
                        help='Fixed/initial Pose.theta (rad) (ignored for uniform) [0.0]')
    parser.add_argument('--nt', type=int, default=1,
                        help='Number of Pose.theta (ignored for uniform) [1]')
    parser.add_argument('--mode', choices=['grid', 'uniform'], default='uniform',
                        help='Sampling mode [uniform]')
    parser.add_argument('--seed', type=int,
                        help='Random number generator seed [None]')

    args = parser.parse_args()
    if len(args.map) <= 0:
      print('Missing --map argument')
      sys.exit(-1)

    # Enforce RNG seed
    if args.seed is not None:
      np.random.seed(args.seed)

    return args


  def __init__(self, args):
    # Initialize node
    self.args = args
    self.aspect_ratio = float(self.args.width)/self.args.height

    # Load map
    print('Loading map %s' % self.args.map)
    self.map = SingleImageServer(imageFilename=self.args.map, ppm=self.args.ppm)

  
  def sample(self, pose_xyz):
    return self.map.getImage(pose_xyz.x, pose_xyz.y, pose_xyz.z, pose_xyz.theta/np.pi*180.,
                             self.args.hfov, self.aspect_ratio, (self.args.height,self.args.width))


  def save_zip(self, zip_path, save_dir, poses):
    # Convert theta as axis-angle to quaternion
    # http://wiki.roblox.com/index.php?title=Quaternions_for_rotation
    qw, qx, qy, qz = np.cos(poses[:, 3]/2), np.sin(poses[:, 3]/2)*0., np.sin(poses[:, 3]/2)*0., np.sin(poses[:, 3]/2)*1.

    # Create Panda data frame
    # NOTE: flip sign of y, so that x and y map to Easting and Northing
    data = []
    for i in tqdm.trange(poses.shape[0], desc='CSV'):
      fname = 'img_%s.png'%i
      data.append([fname, poses[i, 0], -poses[i, 1], poses[i, 2], qw[i], qx[i], qy[i], qz[i], poses[i, 3]])

    # Save metadata csv
    csv_path = os.path.join(save_dir, 'data.csv')
    df = pd.DataFrame(data,columns=['img','x','y','z','q0','q1','q2','q3','theta'])
    df.to_csv(csv_path)

    # Zip temp folder
    zip_name, _ = os.path.splitext(zip_path)
    shutil.make_archive(zip_name, 'zip', save_dir)

    # Remove temp folder
    shutil.rmtree(save_dir)


  def sample_grid(self):
    # Create temp folder to store images and metadata csv
    save_dir = tempfile.mkdtemp()
    os.makedirs(os.path.join(save_dir, 'images'))

    # Initialize tensors
    #images = np.zeros(shape=(self.args.nxyz*self.args.nt, self.args.height, self.args.width, 3), dtype='uint8')
    poses = np.zeros(shape=(self.args.nxyz*self.args.nt, 4), dtype='float64')
    curr_pose = PoseXYZ(z=self.args.z, theta=self.args.t)

    # Determine offset from map boundaries to prevent wrap-around sampling
    image_width_m = (self.args.z+self.args.dz)*2.*np.tan(self.args.hfov/2./180.*np.pi)
    diag_width_scale = np.sqrt(self.aspect_ratio*self.aspect_ratio+1)/self.aspect_ratio
    margin_m = image_width_m*diag_width_scale/2.*1.01 # extra 1% margin

    # Compute number of samples per Cartesian dimension
    # WARNING: if args.nxyz is not a perfect square/cube (w/ dz) then not all grid points will be sampled
    x_min, x_max = margin_m, self.map._width/self.map._pixelsPerMeter - margin_m
    y_min, y_max = margin_m, self.map._height/self.map._pixelsPerMeter - margin_m
    z_min, z_max = self.args.z, self.args.z + self.args.dz
    if self.args.dz > 0:
      ndim = int(np.power(self.args.nxyz, 1/3.)) # more stable than np.cbrt
      if ndim*ndim*ndim < self.args.nxyz:
        ndim += 1
      xyzi = np.unravel_index(range(self.args.nxyz), (ndim, ndim, ndim))
    else:
      ndim = int(np.sqrt(self.args.nxyz))
      if ndim*ndim < self.args.nxyz:
        ndim += 1
      xyzi = np.unravel_index(range(self.args.nxyz), (ndim, ndim))

    # Loop to sample entries
    i = 0
    for ni in tqdm.trange(self.args.nxyz, desc='Sample XYZ'):
      # Sample xyz
      if self.args.dz > 0:
        curr_pose.x = xyzi[0][ni]/(ndim-1.)*(x_max-x_min) + x_min
        curr_pose.y = xyzi[1][ni]/(ndim-1.)*(y_max-y_min) + y_min
        curr_pose.z = xyzi[2][ni]/(ndim-1.)*(z_max-z_min) + z_min
      else: # Sample xy
        curr_pose.x = xyzi[0][ni]/(ndim-1.)*(x_max-x_min) + x_min
        curr_pose.y = xyzi[1][ni]/(ndim-1.)*(y_max-y_min) + y_min

      for nti in range(self.args.nt):
        # Sample theta
        curr_pose.theta = nti*(2*np.pi/self.args.nt)

        # Sample and store
        img = self.sample(curr_pose)
        poses[i, :] = (curr_pose.x, curr_pose.y, curr_pose.z, curr_pose.theta)
        #images[i, :] = img

        fname = 'img_%s.png'%i
        cv2.imwrite(os.path.join(save_dir,'images',fname), img)

        i += 1

    # Save to output file
    self.save_zip(self.args.output, save_dir, poses)
    print('Saved to %s' % self.args.output)


  def sample_uniform(self):
    # Create temp folder to store images and metadata csv
    save_dir = tempfile.mkdtemp()
    os.makedirs(os.path.join(save_dir, 'images'))

    # Initialize tensors
    #images = np.zeros(shape=(self.args.nxyz, self.args.height, self.args.width, 3), dtype='uint8')
    poses = np.zeros(shape=(self.args.nxyz, 4), dtype='float64')
    curr_pose = PoseXYZ(z=self.args.z, theta=self.args.t)

    # Determine offset from map boundaries to prevent wrap-around sampling
    image_width_m = (self.args.z+self.args.dz)*2.*np.tan(self.args.hfov/2./180.*np.pi)
    diag_width_scale = np.sqrt(self.aspect_ratio*self.aspect_ratio+1)/self.aspect_ratio
    margin_m = image_width_m*diag_width_scale/2.*1.01 # extra 1% margin

    # Compute number of samples per Cartesian dimension
    x_min, x_max = margin_m, self.map._width/self.map._pixelsPerMeter - margin_m
    y_min, y_max = margin_m, self.map._height/self.map._pixelsPerMeter - margin_m
    z_min, z_max = self.args.z, self.args.z+self.args.dz

    # Loop to sample entries
    for i in tqdm.trange(self.args.nxyz, desc='Sample'):
      # Sample xyz
      curr_pose.x = np.random.rand()*(x_max-x_min) + x_min
      curr_pose.y = np.random.rand()*(y_max-y_min) + y_min
      curr_pose.z = np.random.rand()*(z_max-z_min) + z_min
      curr_pose.theta = np.random.rand()*2*np.pi

      # Sample and store
      img = self.sample(curr_pose)
      poses[i, :] = (curr_pose.x, curr_pose.y, curr_pose.z, curr_pose.theta)
      #images[i, :] = img

      fname = 'img_%s.png'%i
      cv2.imwrite(os.path.join(save_dir,'images',fname), img)

    # Save to output file
    self.save_zip(self.args.output, save_dir, poses)
    print('Saved to %s' % self.args.output)


if __name__ == '__main__':
  args = PoseSampler.parse_args()
  sampler = PoseSampler(args)
  if args.mode == 'grid':
    sampler.sample_grid()
  elif args.mode == 'uniform':
    sampler.sample_uniform()
  else:
    print('Unrecognized mode: %s' % args.mode)
