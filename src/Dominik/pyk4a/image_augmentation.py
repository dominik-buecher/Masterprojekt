from keras.preprocessing.image import ImageDataGenerator
import cv2
import matplotlib.pyplot as plt
import open3d as o3d
import numpy as np
import os

def augment_and_save_images(input_folder, output_folder):
    # Erstelle den ImageDataGenerator
    datagen = ImageDataGenerator(
        horizontal_flip=True,
    )

    # Erstelle den Ausgabeordner, wenn er noch nicht existiert
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # Liste alle Bilddateien im Eingabeordner auf
    input_files = [f for f in os.listdir(input_folder) if f.endswith(('.png', '.jpg', '.jpeg'))]

    # Augmentiere und speichere jedes Bild
    for input_file in input_files:
        input_path = os.path.join(input_folder, input_file)
        
        # Lade das Bild
        image = cv2.imread(input_path)

        # Füge eine zusätzliche Dimension für den ImageDataGenerator hinzu
        image = np.expand_dims(image, axis=0)

        # Führe die Bildaugmentation durch
        augmented_images = datagen.flow(image, batch_size=1)

        # Erstelle den Dateinamen für das augmentierte Bild
        output_file = f"augmented_{input_file}"

        # Erstelle den Pfad zum Ausgabebild
        output_path = os.path.join(output_folder, output_file)

        # Speichere das augmentierte Bild
        augmented_image = next(augmented_images)[0].astype(np.uint8)
        cv2.imwrite(output_path, augmented_image)




def mirror_point_cloud(input_path, output_path):
    # Lade die Point Cloud
    pcd = o3d.io.read_point_cloud(input_path)

    # Erhalte die Punkte und Farben
    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)

    # Spiegele die Punkte an der X-Achse (horizontal)
    points[:, 0] = -points[:, 0]

    # Erstelle eine neue Point Cloud mit den gespiegelten Punkten und Farben
    mirrored_pcd = o3d.geometry.PointCloud()
    mirrored_pcd.points = o3d.utility.Vector3dVector(points)
    mirrored_pcd.colors = o3d.utility.Vector3dVector(colors)

    # Speichere die gespiegelte Point Cloud
    o3d.io.write_point_cloud(output_path, mirrored_pcd)

def mirror_point_clouds_in_folder(input_folder, output_folder):
    # Erstelle den Ausgabeordner, wenn er noch nicht existiert
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # Liste alle .ply-Dateien im Eingabeordner auf
    input_files = [f for f in os.listdir(input_folder) if f.endswith(".ply")]

    # Spiegle jede .ply-Datei
    for input_file in input_files:
        input_path = os.path.join(input_folder, input_file)
        output_file = f"augmented_{input_file}"
        output_path = os.path.join(output_folder, output_file)
        mirror_point_cloud(input_path, output_path)

if __name__ == "__main__":
    # Eingabe- und Ausgabeordner festlegen
    input_folder_path = "dataset/original_images/pointcloud"
    output_folder_path = "dataset/augmented_images/pointcloud"

    # Spiegle alle .ply-Dateien im Eingabeordner und speichere sie im Ausgabeordner
    #mirror_point_clouds_in_folder(input_folder_path, output_folder_path)


    # Color images
    input_folder_path = r"dataset\original_images\color_images"
    output_folder_path = r"dataset\augmented_images\color_images"

    # Augmentiere alle Bilder im Eingabeordner und speichere sie im Ausgabeordner
    #augment_and_save_images(input_folder_path, output_folder_path)

    # depth images
    input_folder_path1 = r"dataset\original_images\depth_images\colorized"
    output_folder_path1 = r"dataset\augmented_images\depth_images\colorized"
    
    input_folder_path2 = r"dataset\original_images\depth_images\normal"
    output_folder_path2 = r"dataset\augmented_images\depth_images\normal"
    
    #augment_and_save_images(input_folder_path1, output_folder_path1)
    #augment_and_save_images(input_folder_path2, output_folder_path2)

    # Infrared images
    input_folder_path_ir = r"dataset\original_images\infrared_images"
    output_folder_path_ir = r"dataset\augmented_images\infrared_images"

    # Augmentiere alle Bilder im Eingabeordner und speichere sie im Ausgabeordner
    #augment_and_save_images(input_folder_path_ir, output_folder_path_ir)

    
    # single grape images
    input_folder_path = r"dataset\single_grape\original"
    output_folder_path = r"dataset\single_grape\augmented"

    # Augmentiere alle Bilder im Eingabeordner und speichere sie im Ausgabeordner
    augment_and_save_images(input_folder_path, output_folder_path)
