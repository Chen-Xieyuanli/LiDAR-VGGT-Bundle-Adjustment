# LiDAR-VGGT-Bundle-Adjustment
This repository is built on [BALM2](https://github.com/hku-mars/BALM), and is an enhanced version for cross-modal LiDAR and VGGT to jointly optimize the poses of LiDAR and VGGT based on the point cloud association.  It is worth noting that the optimization here is not based on optimizing image feature points like visual BA, but rather on the geometric structure of point clouds.



## Concerns

* The aim is to adjust the poses of VGGT and improve the point cloud geometry accuracy based on precise structures by LiDAR, however, due to the poor geometry structure VGGT provided such as curved planes,  the voxelization and data association may be severely impacted.
* The optimization may destroy the texture accuracy of VGGT point cloud, leading to blurred visual presentation.
* Perhaps a better optimization approach is needed, directly optimizing the VGGT point cloud position instead of assuming the point cloud points and LiDAR are fixed, as is the case with LiDAR BALM.