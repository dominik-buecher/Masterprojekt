import open3d as o3d
import numpy as np

def region_growing_radius(pcd, seed_index, radius_threshold):
    """
    Region Growing basierend auf Radien.
    
    Parameters:
    - pcd: Punktwolke (PointCloud)
    - seed_index: Index des Samenpunkts (int)
    - radius_threshold: Radius-Schwellenwert für das Region Growing (float)
    
    Returns:
    - region: Indizes der Punkte in der Region (Liste von ints)
    """
    # Konvertieren Sie die Punktwolke in ein NumPy-Array
    points = np.asarray(pcd.points)

    # Initialisieren Sie die Region mit dem Samenpunkt
    region = [seed_index]

    # Erstellen Sie eine Warteschlange für das Region Growing
    queue = [seed_index]

    while queue:
        current_index = queue.pop(0)

        # Finden Sie die Nachbarn des aktuellen Punktes
        distances = np.linalg.norm(points - points[current_index], axis=1)
        neighbors = np.where(distances < radius_threshold)[0]

        # Fügen Sie benachbarte Punkte zur Region hinzu
        for neighbor_index in neighbors:
            if neighbor_index not in region:
                region.append(neighbor_index)
                queue.append(neighbor_index)

    return region

# Beispielverwendung:
file_path = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\data\temp\pointcloud_cropped.ply"
pcd = o3d.io.read_point_cloud(file_path)

# Wählen Sie einen Samenpunkt (hier: den ersten Punkt)
seed_index = 100

# Setzen Sie den Radius-Schwellenwert (anpassen nach Bedarf)
radius_threshold = 0.01  # Hier: 1 cm

# Wenden Sie das Region Growing an
result_region = region_growing_radius(pcd, seed_index, radius_threshold)

# Visualisieren Sie die Ergebnisse
pcd_result = o3d.geometry.PointCloud()
pcd_result.points = o3d.utility.Vector3dVector(np.asarray(pcd.points)[result_region])

o3d.visualization.draw_geometries([pcd_result])
