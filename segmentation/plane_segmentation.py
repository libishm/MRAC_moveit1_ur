#3D point cloud segmentation based of normals and distances,
​
import cv2
import numpy as np
import open3d as o3d
import matplotlib.colors as colors
import matplotlib.cm as cmx
import matplotlib.pyplot as plt
from statistics import mean
​
from open3d import *
def get_vector_angle(u,v):
    #print(u,v)
    uv=np.dot(u,v)
    #mu=np.linalg.norm(u)
    #mv=np.linalg.norm(v)
    ang = np.arccos(uv)
    ang = ang*180/np.pi
    if (ang > 90):
        ang = 180 - ang
    return (ang)
def get_plane_distance(n,p):
    d=np.dot(n,p)
    return (d)
​
def get_distance_group(dist,group1,distance_resolution):
    group={}
    group_center={}
    group[0]=[group1[0]]
    group_center[0]=dist[0]
    u=dist[0]
    
    for i in np.arange(1,len(group1)):
        v=dist[i]
        for y in group_center: #Add the new member in the matching group
            u=group_center[y]
            if abs(u-v) <=distance_resolution:
                group[y] +=[group1[i]]
                break
        else: #Create a new group center
            new_group=len(group_center)
            group_center[new_group]=v
            group[new_group]=[group1[i]]
    return (group, group_center)
​
​
if __name__ == "__main__":
    angle_resolution=10
    distance_resolution=0.05
    thresold=150
    print("Load a ply point cloud and render it")
    pcd = o3d.io.read_point_cloud("/home/libish/granite_shards.ply")
    
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.00909,
                                             ransac_n=5,
                                             num_iterations=1000)
    [a, b, c, d] = plane_model
    print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
    print("Displaying pointcloud with planar points in red ...")
    inlier_cloud = pcd.select_by_index(inliers)
    inlier_cloud.paint_uniform_color([1.0, 0, 0])
​
    pcd = pcd.select_by_index(inliers, invert=True)
    o3d.visualization.draw([inlier_cloud, pcd])
    print("Downsample the point cloud with a voxel of 0.05")
    pcd = pcd.voxel_down_sample(voxel_size=0.005)
    o3d.visualization.draw([pcd])
​
    normals= np.asarray(pcd.normals)
    group={}
    group_center={}
    group[0]=[0]
    group_center[0]=normals[0]
    u=normals[0]
    for i in np.arange(1,len(normals)):
        v=normals[i]
        for y in group_center: #Add the new member in the matching group
            u=group_center[y]
            if get_vector_angle(u,v) <=angle_resolution:
                group[y] +=[i]
                break
        else: #Create a new group center
            new_group=len(group_center)
            group_center[new_group]=v
            group[new_group]=[i]
​
​
    planes={}
    no_of_planes=0
    for i in group_center:        
        n=group_center[i]
        dist=[]
        for idx in group[i]:
            p= pcd.points[idx]
            dist+=[get_plane_distance(n,p)]
         ##        plt.plot(np.sort(dist))
         ##        plt.show()
        a,b=get_distance_group(dist,group[i],distance_resolution)
        planes[i]=[a]
        no_of_planes+=len(b)
​
    jet=plt.get_cmap('jet')
    cNorm = colors.Normalize(vmin=0.1,vmax=10*no_of_planes)
    scalarMap=cmx.ScalarMappable(norm=cNorm,cmap=jet)
​
    current_plane_no=0
    no_large_planes=0
    for i in planes:
        p=planes[i]
        for idx in p[0]:
            a=p[0][idx]
            if len(a) <thresold:
                np.asarray(pcd.colors)[a[:],:]=[1,1,1]
                current_plane_no+=1
                continue
            plane_color=np.random.randint(0.09,10*no_of_planes)
            print(plane_color)
            colorValue=scalarMap.to_rgba(plane_color)
            np.asarray(pcd.colors)[a[:],:]=list(colorValue[0:3])
            current_plane_no+=1
            no_large_planes+=1
            o3d.visualization.draw_geometries([pcd])
​
            # dbscan clustering on pointcloud
            cluster = pcd.cluster_dbscan(eps=0.02, min_points=10, print_progress=True)
            o3d.visualization.draw_geometries([pcd])
​
​
#create mesh from point cloud
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, 0.005)
    mesh.compute_vertex_normals()
    o3d.visualization.draw_geometries([mesh])
​
o3d.write_image("/home/libish/pointcloud.png", o3d.geometry.Image(np.asarray(pcd.colors)))
​
    # traingle_mesh_to_shapely_linestring
    
    # with o3d.utility.VerbosityContextManager( 
    #         o3d.utility.VerbosityLevel.Debug) as cm:
    #     labels = np.array(
    #         pcd.cluster_dbscan(eps=0.0155, min_points=150, print_progress=True))
​
    # max_label = labels.max()
    # print(f"point cloud has {max_label + 1} clusters")
    # colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    # colors[labels < 0] = 0
    # pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
    # o3d.visualization.draw([pcd])
