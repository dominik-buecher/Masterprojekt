import open3d as o3d

def read_ply_file(file_path):
    # Einlesen der PLY-Datei
    pcd = o3d.io.read_point_cloud(file_path)
    
    # Extrahieren der XYZ-Koordinaten
    xyz_coords = pcd.points
    
    # Ausgabe der XYZ-Koordinaten
    for point in xyz_coords:
        print(point[0])

if __name__ == "__main__":
    # Pfad zur PLY-Datei angeben
    ply_file_path = r'C:\Users\Dominik\Documents\Studium\Master\Masterprojekt\Masterprojekt\src\CloudComPy\dataset\quality_good\cropped_pointcloud\pointcloud_copped_1.ply'
    
    # PLY-Datei einlesen und XYZ-Koordinaten ausgeben
    read_ply_file(ply_file_path)
