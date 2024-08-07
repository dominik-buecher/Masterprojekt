import os

def count_ply_files_in_directory(directory):
    return len([f for f in os.listdir(directory) if f.endswith('.ply')])

def process_directory_structure(base_path, output_file):
    with open(output_file, 'w') as output:
        i = 1
        while True:
            # Erstelle den Namen des Ordners, der hochgezählt wird
            pointcloud_dir = os.path.join(base_path, f'pointcloud_copped_{i}.ply')
            mesh_dir = os.path.join(pointcloud_dir, 'mesh')
            mesh_cloud_color_dir = os.path.join(pointcloud_dir, 'mesh_cloud_color_green')

            # Überprüfe, ob der Ordner existiert
            if not os.path.exists(pointcloud_dir):
                print(f"Ordner {pointcloud_dir} existiert nicht. Beende die Schleife.")
                break

            # Zähle die .ply Dateien in den Verzeichnissen
            mesh_ply_count = count_ply_files_in_directory(mesh_dir)
            mesh_cloud_color_ply_count = count_ply_files_in_directory(mesh_cloud_color_dir)

            # Schreibe die Ergebnisse in die Ausgabedatei
            output.write(f'pointcloud_copped_{i}.ply: - mesh: {mesh_ply_count} .ply files, mesh_cloud_color_green: {mesh_cloud_color_ply_count} .ply files\n')

            print(f"Verarbeitet: {pointcloud_dir}")
            
            # Inkrementiere den Zähler für den nächsten Ordner
            i += 1
            if i == 100:
                break

if __name__ == "__main__":
    base_path = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\dataset\quality_good\meshes"
    output_file = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\dataset\quality_good\ply_counts.txt"

    process_directory_structure(base_path, output_file)
