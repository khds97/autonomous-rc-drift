#!/usr/bin/env python3
"""Camera intrinsic calibration from checkerboard photos (OpenCV).

Take 20 to 40 pictures of a printed checkerboard with the fixed camera at the same
resolution the VIO uses. Tilt and move the board so it covers the whole image, including
the corners, and keep it flat and still while taking each picture.

    python3 calibrate_intrinsics.py --images ~/calib --cols 8 --rows 6 --square 0.025

cols and rows are the number of inner corners (a board with 9 x 7 squares has 8 x 6).
square is the side length of one square in meters. The result is printed as a block for
config/ground_vio.yaml (k1, k2, p1, p2 with k3 fixed to zero, which suits small lenses).
"""
import argparse
import glob
import os
import sys

import cv2
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--images', required=True, help='folder with the checkerboard pictures')
    parser.add_argument('--cols', type=int, required=True, help='inner corners per row')
    parser.add_argument('--rows', type=int, required=True, help='inner corners per column')
    parser.add_argument('--square', type=float, required=True, help='square size in meters')
    parser.add_argument('--show', action='store_true', help='show the detected corners')
    args = parser.parse_args()

    files = sorted(
        f for ext in ('png', 'jpg', 'jpeg', 'bmp', 'pgm')
        for f in glob.glob(os.path.join(os.path.expanduser(args.images), '*.' + ext)))
    if not files:
        sys.exit('no pictures found in %s' % args.images)

    pattern = (args.cols, args.rows)
    grid = np.zeros((args.cols * args.rows, 3), np.float32)
    grid[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square

    object_points, image_points, used_files = [], [], []
    size = None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-4)
    for path in files:
        gray = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if gray is None:
            print('skip (unreadable): %s' % path)
            continue
        if size is None:
            size = (gray.shape[1], gray.shape[0])
        elif size != (gray.shape[1], gray.shape[0]):
            print('skip (different size): %s' % path)
            continue
        found, corners = cv2.findChessboardCorners(
            gray, pattern, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not found:
            print('skip (no board found): %s' % path)
            continue
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        object_points.append(grid)
        image_points.append(corners)
        used_files.append(path)
        if args.show:
            vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            cv2.drawChessboardCorners(vis, pattern, corners, found)
            cv2.imshow('corners', vis)
            cv2.waitKey(200)

    used = len(object_points)
    print('%d of %d pictures used, image size %dx%d' % (used, len(files), size[0], size[1]))
    if used < 10:
        sys.exit('need at least 10 usable pictures (20 or more is better)')

    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        object_points, image_points, size, None, None, flags=cv2.CALIB_FIX_K3)

    # Reprojection error per picture, to find bad ones.
    errors = []
    for obj, img, rvec, tvec in zip(object_points, image_points, rvecs, tvecs):
        projected, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
        diff = projected.reshape(-1, 2) - img.reshape(-1, 2)   # corner arrays differ between OpenCV versions
        errors.append(float(np.sqrt(np.mean(np.sum(diff ** 2, axis=1)))))
    worst = int(np.argmax(errors))

    print('RMS reprojection error: %.3f px, worst picture %.3f px (%s)' %
          (rms, errors[worst], os.path.basename(used_files[worst])))
    if rms > 0.8:
        print('WARNING: the error is high. Remove blurred pictures or ones with a bent board.')
    spread = np.ptp(np.vstack([p.reshape(-1, 2) for p in image_points]), axis=0)
    if spread[0] < 0.8 * size[0] or spread[1] < 0.8 * size[1]:
        print('WARNING: the board never came close to the image edges. '
              'The distortion is poorly determined there.')

    d = dist.ravel()
    print('\n# paste into config/ground_vio.yaml, then set calibrated: true')
    print('    camera:')
    print('      calibrated: true')
    print('      fx: %.3f' % K[0, 0])
    print('      fy: %.3f' % K[1, 1])
    print('      cx: %.3f' % K[0, 2])
    print('      cy: %.3f' % K[1, 2])
    print('      distortion: [%.6f, %.6f, %.6f, %.6f]' % (d[0], d[1], d[2], d[3]))


if __name__ == '__main__':
    main()
