import os
import shutil

output_path = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\dataset\quality_bad\mesh_cloud"

for i in range(1, 61):
    # Erzeuge den Pfad zur Quelldatei
    source_dir = fr"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\dataset\quality_bad\meshes\pointcloud_copped_{i}.ply\mesh_cloud_color"
    
    # Suche nach allen .ply-Dateien im Quellverzeichnis
    if os.path.exists(source_dir):
        for file_name in os.listdir(source_dir):
            if file_name.endswith('.ply'):
                source_file = os.path.join(source_dir, file_name)
                
                # Erzeuge den neuen Dateinamen mit _i vor .ply
                base_name = os.path.splitext(file_name)[0]
                new_file_name = f"{base_name}_{i}.ply"
                destination_file = os.path.join(output_path, new_file_name)
                
                # Kopiere die Datei in das Zielverzeichnis
                shutil.copy2(source_file, destination_file)
                print(f"Kopiere {source_file} nach {destination_file}")
    else:
        print(f"Verzeichnis {source_dir} existiert nicht.")
