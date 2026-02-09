# LiDAR and VGGT Cross-Modal BA
## 方案一
* 最底层逐个chunk内所有LiDAR帧和VGGT相机帧率做BA     **(最底层尺度的跨模态融合)**       直接使用/修改BALM代码    
* 仅仅聚合VGGT的点云到每个chunk第一帧               **(恢复到chunk层级操作)**
* icp获取相邻的VGGT chunk之间的overlap 区域的约束，构建位姿图优化    **(chunk对齐融合)**
* 所有chunk再跑HBA 两层layer **(可选)**    
## 方案二
* 最底层逐个chunk内所有LiDAR帧和VGGT相机帧率做BA     **(最底层尺度的跨模态融合)**       直接使用/修改BALM代码    
* 仅仅聚合VGGT的点云到每个chunk第一帧               **(恢复到chunk层级操作)**
* 所有chunk再跑HBA 两层layer 只BA不PGO
* icp获取相邻的VGGT chunk之间的overlap 区域的约束，构建位姿图优化，结合HBA的约束    **(chunk对齐融合)**
    