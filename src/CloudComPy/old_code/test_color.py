import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

file_path1 = fr"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\data\mesh_cloud_3.ply"

point_cloud = o3d.io.read_point_cloud(file_path1)

points = np.asarray(point_cloud.points)
colors_float = np.asarray(point_cloud.colors)
colors_int = (colors_float * 255).astype(np.uint8)

print("colors: ", colors_int)
color = colors_int[1]
image = np.zeros((100, 100, 3), dtype=np.uint8)
image[:, :] = color

# Zeige das Bild mit Matplotlib an
plt.imshow(image)
plt.show()


def is_green(color):
    r, g, b = color
    
    # Definiere einen Bereich für die Grünkomponente
    # Der Bereich kann angepasst werden, um den gewünschten Grünton abzudecken
    green_range_min = 50
    green_range_max = 255
    
    # Prüfe, ob die Grünkomponente innerhalb des definierten Bereichs liegt
    # und ob sie die stärkste Komponente ist
    if green_range_min <= g <= green_range_max and g > r and g > b:
        print("Die Farbe ist grün.")
    else:
        print("Die Farbe ist nicht grün.")

# Beispiel-RGB-Farbe
color = (34, 139, 34)  # Ein Grünton

# Funktion aufrufen, um zu prüfen, ob die Farbe grün ist
is_green(color)
