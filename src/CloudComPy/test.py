
import os
import sys
import math

#os.environ["_CCTRACE_"]="ON" # only if you want C++ debug traces

from gendata import dataDir, isCoordEqual
import cloudComPy as cc

# #---RANSACSD01-begin
# tr0 = cc.ccGLMatrix()
# tr0.initFromParameters(math.pi/3., (0., 1., 0.), (3.0, 0.0, 4.0))
# cylinder = cc.ccCylinder(0.5, 2.0, tr0)
# c0 = cylinder.samplePoints(True, 1000)

# tr1 = cc.ccGLMatrix()
# tr1.initFromParameters(0.0, (0., 0., 0.), (-2.0, 5.0, 1.0))
# sphere1 = cc.ccSphere(1.5, tr1)
# c1 = sphere1.samplePoints(True, 1000)

# tr2 = cc.ccGLMatrix()
# tr2.initFromParameters(0.0, (0., 0., 0.), (6.0, -3.0, -2.0))
# sphere2 = cc.ccSphere(2.0, tr2)
# c2 = sphere2.samplePoints(True, 1000)

# tr3 = cc.ccGLMatrix()
# tr3.initFromParameters(0.0, (0., 0., 0.), (0.0, 1.0, 2.0))
# sphere3 = cc.ccSphere(1.0, tr3)
# c3 = sphere3.samplePoints(True, 1000)

# cloud = c0.cloneThis()
# cloud.fuse(c1)
# cloud.fuse(c2)
# cloud.fuse(c3)
# #---RANSACSD01-end


# RansacParams.allowFitting                     --> Use Least Squares fitting on found shapes, default True.
# RansacParams.allowSimplification              --> Attempt to simplify shapes. Will attempt to convert torus, cones, spheres, cylinders into simpler shapes. For instance, cones may be simplified into cylinder, sphere or plane. Default True.
# RansacParams.bitmapEpsilon                    --> Sampling resolution, should correspond to the average distance of neighbors points in data, default 0.001.
# RansacParams.createCloudFromLeftOverPoints    --> Save leftover points into a new cloud, default True.
# RansacParams.epsilon                          --> Maximum distance if the samples to the ideal shape, default 0.005.
# RansacParams.getPrimEnabled()                 --> Check the type of primitive (plane, sphere, cylinder, cone, torus) enabled. By default, plane, sphere, cylinder are enabled. Types of primitive are from: RANSAC_PRIMITIVE_TYPES.RPT_SPHERE
# RansacParams.maxConeAngle_deg                 --> Maximum cone angle, default +infinity.
# RansacParams.maxConeLength                    --> Maximum cone length, default +infinity.
# RansacParams.maxConeRadius                    --> Maximum cone radius, default +infinity.
# RansacParams.maxCylinderLength                --> Maximum cylinder length, default +infinity.
# RansacParams.maxCylinderRadius                --> Maximum cylinder radius, default +infinity.
# RansacParams.maxNormalDev_deg                 --> Maximum deviation from the ideal shape normal vector (in degrees), default 25.

# RansacParams.maxSphereRadius                  --> Maximum sphere radius, default +infinity.

# RansacParams.maxTorusMajorRadius              --> Maximum tore major radius, default +infinity.
# RansacParams.maxTorusMinorRadius              --> Maximum tore minor radius, default +infinity.
# RansacParams.minCylinderRadius                --> Minimum cylinder radius, default -infinity.

# RansacParams.minSphereRadius                  --> Minimum sphere radius, default -infinity.

# RansacParams.minTorusMajorRadius              --> Minimum tore major radius, default -infinity.
# RansacParams.minTorusMinorRadius              --> Minimum tore minor radius, default -infinity.

# RansacParams.optimizeForCloud()               --> Sets values for epsilon and bitmapEpsilon based on the cloud bounding box, maximum dimension. epsilon is set to 0.005 * maxDim, bitmapEpsilon is set to 0.01 * maxDim.
# RansacParams.probability                      --> Probability that no better candidate was overlooked during the sampling, the lower the better, default 0.01
# RansacParams.randomColor                      --> Set random color for each shape found, default True.
# RansacParams.setPrimEnabled()                 --> Define the type of primitive (plane, sphere, cylinder, cone, torus) enabled. By default, plane, sphere, cylinder are enabled. Types of primitive are from: RANSAC_PRIMITIVE_TYPES.RPT_SPHERE
# RansacParams.supportPoints                    --> Minimal number of points required to define a primitive, default 500.


isRANSAC_SD_available = cc.isPluginRANSAC_SD()
#print("isRANSAC_SD_available: ", isRANSAC_SD_available)

#cloud = cc.loadPointCloud("data/temp/pointcloud_cropped.ply")
cloud = cc.loadPointCloud(r"C:\Users\Dominik\Documents\Studium\Master\Masterprojekt\Masterprojekt\data\temp\pointcloud_cropped.ply")
print("cloud: ", cloud)

#---RANSACSD02-begin
if cc.isPluginRANSAC_SD():
    print("isRANSAC_SD_available: ", isRANSAC_SD_available)
    import cloudComPy.RANSAC_SD
    print("cloudComPy.RANSAC_SD: ")
    params = cc.RANSAC_SD.RansacParams()
    params.maxSphereRadius = 2
    params.minSphereRadius = 0.5
    print("params: ", params)
    params.optimizeForCloud(cloud)
    print(params.epsilon, params.bitmapEpsilon)
    meshes, clouds = cc.RANSAC_SD.computeRANSAC_SD(cloud, params)
    print("meshes: ", meshes)
    print("clouds: ", clouds)
    print("finished!")
    #---RANSACSD02-end
    
    sortmeshes = [m for m in meshes if m is not None]
    sortmeshes.sort(key= lambda m: m.getName())
    for i in range(len(sortmeshes)):
        print("mesh %d: %s"%(i,sortmeshes[i].getName()))
    
    if not sortmeshes[0].isA(cc.CC_TYPES.CYLINDER):
        print("sortmeshes[0]")
        raise RuntimeError
    if not sortmeshes[1].isA(cc.CC_TYPES.PLANE):
        print("sortmeshes[1]")
        raise RuntimeError
    if not sortmeshes[2].isA(cc.CC_TYPES.PLANE):
        print("sortmeshes[2]")
        raise RuntimeError
    if not sortmeshes[3].isA(cc.CC_TYPES.SPHERE):
        print("sortmeshes[3]")
        raise RuntimeError
    if not sortmeshes[4].isA(cc.CC_TYPES.SPHERE):
        print("sortmeshes[4]")
        raise RuntimeError
    if not sortmeshes[5].isA(cc.CC_TYPES.SPHERE):
        print("sortmeshes[5]")
        raise RuntimeError
    
    if not math.isclose(sortmeshes[3].getRadius(), 1.0, rel_tol=3.e-2):
        print("isclose(sortmeshes[3]")
        raise RuntimeError
    if not math.isclose(sortmeshes[4].getRadius(), 1.5, rel_tol=3.e-2):
        print("isclose(sortmeshes[4]")
        raise RuntimeError
    if not math.isclose(sortmeshes[5].getRadius(), 2.0, rel_tol=3.e-2):
        print("isclose(sortmeshes[5]")
        raise RuntimeError
    
    tr_0 = sortmeshes[0].getTransformation()
    t3D0 = tr_0.getParameters1().t3D
    if not isCoordEqual(t3D0, (3.0, 0.0, 4.0), 0, 3.e-2):
        print(t3D0)
        raise RuntimeError
    tr_3 = sortmeshes[3].getTransformation()
    t3D3 = tr_3.getParameters1().t3D
    if not isCoordEqual(t3D3, (0.0, 1.0, 2.0), 0, 3.e-2):
        print(t3D3)
        raise RuntimeError
    tr_4 = sortmeshes[4].getTransformation()
    t3D4 = tr_4.getParameters1().t3D
    if not isCoordEqual(t3D4, (-2.0, 5.0, 1.0), 3.e-2):
        print(t3D4)
        raise RuntimeError
    tr_5 = sortmeshes[5].getTransformation()
    t3D5 = tr_5.getParameters1().t3D
    if not isCoordEqual(t3D5, (6.0, -3.0, -2.0), 3.e-2):
        print(t3D5)
        raise RuntimeError
    
    
    shapes = [cloud] #cylinder, sphere1, sphere2, sphere3, 
    for mesh in meshes:
        if mesh is not None:
            print("mesh:", mesh.getName())
            shapes.append(mesh)
    for cl in clouds:
        if cl is not None:
            print("cloud:", cl.getName())
            shapes.append(cl)
    
    cc.SaveEntities(shapes, os.path.join(dataDir, "ransac.bin"))


    for i, sphere_cloud in enumerate(clouds):
        if sphere_cloud is not None and sortmeshes[i + 3].isA(cc.CC_TYPES.SPHERE):
            sphere_cloud_name = f"sphere_{i + 1}.ply"
            sphere_cloud_path = os.path.join("src/Dominik/CloudComPy", sphere_cloud_name)
            print("saved!")
            sphere_cloud.saveToFile(sphere_cloud_path)