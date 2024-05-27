import numpy as np
import os
from sklearn.model_selection import train_test_split
import tensorflow as tf
from tensorflow.keras import layers, models

# Funktion zum Lesen der Daten aus den Textdateien
def read_data(directory):
    data = []
    labels = []
    
    for filename in os.listdir(directory):
        if filename.endswith(".txt"):
            filepath = os.path.join(directory, filename)
            with open(filepath, 'r') as file:
                lines = file.readlines()
                features = [tuple(map(float, line.split()[:4])) for line in lines]  # Annahme: Farb- und Distanzdaten sind die ersten vier Werte
                data.append(features)
                label = 1 if "good" in filename else 0
                labels.append(label)
                
    return data, labels




# Datenverzeichnis festlegen
good_trauben_dir = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\dataset\quality_good\distance"
bad_trauben_dir = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt\src\CloudComPy\dataset\quality_bad\distance"

good_trauben_data, good_trauben_labels = read_data(good_trauben_dir)
bad_trauben_data, bad_trauben_labels = read_data(bad_trauben_dir)

# Daten kombinieren
all_data = good_trauben_data + bad_trauben_data
all_labels = good_trauben_labels + bad_trauben_labels

# In NumPy-Arrays umwandeln
all_data = np.array(all_data)
all_labels = np.array(all_labels)

# Trainings- und Testdaten aufteilen
train_data, test_data, train_labels, test_labels = train_test_split(all_data, all_labels, test_size=0.2, random_state=42)

# Modell erstellen
model = models.Sequential([
    layers.Flatten(input_shape=(train_data.shape[1], train_data.shape[2])),
    layers.Dense(128, activation='relu'),
    layers.Dense(64, activation='relu'),
    layers.Dense(1, activation='sigmoid')
])

# Modell kompilieren
model.compile(optimizer='adam',
              loss='binary_crossentropy',
              metrics=['accuracy'])

# Modell trainieren
model.fit(train_data, train_labels, epochs=10, batch_size=32, validation_split=0.2)

# Modell evaluieren
test_loss, test_acc = model.evaluate(test_data, test_labels)
print('Test accuracy:', test_acc)
