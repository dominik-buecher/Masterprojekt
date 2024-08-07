import open3d as o3d
import numpy as np
import os
from sklearn.model_selection import train_test_split
import tensorflow as tf
from tensorflow.keras import layers, models
import matplotlib.pyplot as plt
from sklearn.metrics import roc_curve, auc, confusion_matrix, ConfusionMatrixDisplay

def read_point_cloud(ply_file):
    pcd = o3d.io.read_point_cloud(ply_file)
    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)
    return np.hstack((points, colors))

def load_data(directory, label):
    data = []
    labels = []
    for filename in os.listdir(directory):
        if filename.endswith(".ply"):
            filepath = os.path.join(directory, filename)
            data_points = read_point_cloud(filepath)
            data.append(data_points)
            labels.append(label)
    return data, labels

def prepare_data(positive_dir, negative_dir):
    positive_data, positive_labels = load_data(positive_dir, 1)
    negative_data, negative_labels = load_data(negative_dir, 0)

    all_data = positive_data + negative_data
    all_labels = positive_labels + negative_labels

    # Normalisierung und Padding der Daten
    max_points = max(len(pc) for pc in all_data)
    print("max_points: ", max_points)
    for i in range(len(all_data)):
        pc = all_data[i]
        padded_pc = np.pad(pc, ((0, max_points - pc.shape[0]), (0, 0)), mode='constant')
        all_data[i] = padded_pc

    all_data = np.array(all_data)
    all_labels = np.array(all_labels)
    return all_data, all_labels, max_points

def build_pointnet_model(input_shape):
    inputs = layers.Input(shape=input_shape)

    # Input Transformation Network
    x = layers.Conv1D(64, 1, activation='relu')(inputs)
    x = layers.Conv1D(128, 1, activation='relu')(x)
    x = layers.Conv1D(1024, 1, activation='relu')(x)
    x = layers.GlobalMaxPooling1D()(x)
    x = layers.Dense(512, activation='relu')(x)
    x = layers.Dense(256, activation='relu')(x)
    x = layers.Dense(input_shape[1] * input_shape[1], activation='relu')(x)
    input_transform = layers.Reshape((input_shape[1], input_shape[1]))(x)

    transformed_inputs = layers.Dot(axes=(2, 1))([inputs, input_transform])

    # Feature Extraction Network
    x = layers.Conv1D(64, 1, activation='relu')(transformed_inputs)
    x = layers.Conv1D(64, 1, activation='relu')(x)
    x = layers.Conv1D(64, 1, activation='relu')(x)
    x = layers.Conv1D(128, 1, activation='relu')(x)
    x = layers.Conv1D(1024, 1, activation='relu')(x)
    x = layers.GlobalMaxPooling1D()(x)

    # Classification Network
    x = layers.Dense(512, activation='relu')(x)
    x = layers.Dense(256, activation='relu')(x)
    x = layers.Dense(128, activation='relu')(x)
    x = layers.Dense(1, activation='sigmoid')(x)

    model = models.Model(inputs=inputs, outputs=x)

    model.compile(optimizer='adam',
                  loss='binary_crossentropy',
                  metrics=['accuracy'])
    return model

def plot_training_history(history, version):
    plt.figure(figsize=(12, 4))
    plt.subplot(1, 2, 1)
    plt.plot(history.history['accuracy'])
    plt.plot(history.history['val_accuracy'])
    plt.title('Model accuracy')
    plt.ylabel('Accuracy')
    plt.xlabel('Epoch')
    plt.legend(['Train', 'Validation'], loc='upper left')
    plt.savefig(rf'C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\models\images\training_accuracy_v{version}.png')

    plt.subplot(1, 2, 2)
    plt.plot(history.history['loss'])
    plt.plot(history.history['val_loss'])
    plt.title('Model loss')
    plt.ylabel('Loss')
    plt.xlabel('Epoch')
    plt.legend(['Train', 'Validation'], loc='upper left')
    plt.savefig(rf'C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\models\images\training_loss_v{version}.png')

    plt.show()

def plot_roc_curve(test_labels, test_predictions, version):
    fpr, tpr, _ = roc_curve(test_labels, test_predictions)
    roc_auc = auc(fpr, tpr)
    
    plt.figure()
    plt.plot(fpr, tpr, color='darkorange', lw=2, label='ROC curve (area = %0.2f)' % roc_auc)
    plt.plot([0, 1], [0, 1], color='navy', lw=2, linestyle='--')
    plt.xlim([0.0, 1.0])
    plt.ylim([0.0, 1.05])
    plt.xlabel('False Positive Rate')
    plt.ylabel('True Positive Rate')
    plt.title('Receiver Operating Characteristic')
    plt.legend(loc="lower right")
    plt.savefig(rf'C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\models\images\roc_curve_v{version}.png')
    plt.show()

def plot_confusion_matrix(test_labels, test_predictions, version):
    cm = confusion_matrix(test_labels, test_predictions)
    disp = ConfusionMatrixDisplay(confusion_matrix=cm, display_labels=[False, True])
    disp.plot(cmap=plt.cm.Blues)
    plt.title('Confusion Matrix')
    plt.savefig(rf'C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\models\images\confusion_matrix_v{version}.png')
    plt.show()

def main():
    version = 6
    
    positive_dir = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\dataset\quality_good\distance"
    negative_dir = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\dataset\quality_bad\distance"

    all_data, all_labels, max_points = prepare_data(positive_dir, negative_dir)
    
    train_data, test_data, train_labels, test_labels = train_test_split(all_data, all_labels, test_size=0.2, random_state=42)

    input_shape = (max_points, 6)  # 3 für die Positionen und 3 für die Farben

    model = build_pointnet_model(input_shape)

    history = model.fit(train_data, train_labels, epochs=20, batch_size=32, validation_split=0.3)
    
    # Modell speichern
    model.save(fr'C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\models\pointnet_model_kartesisch_v{version}.h5')
    model.summary()
    test_loss, test_acc = model.evaluate(test_data, test_labels)
    print('Test accuracy:', test_acc)

    # Vorhersagen für ROC-Kurve und Verwirrungsmatrix
    test_predictions = (model.predict(test_data) > 0.8).astype("int32")
    
    plot_training_history(history, version)
    plot_roc_curve(test_labels, test_predictions, version)
    plot_confusion_matrix(test_labels, test_predictions, version)

if __name__ == "__main__":
    main()
