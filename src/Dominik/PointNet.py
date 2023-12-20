import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, Dataset
from sklearn.model_selection import train_test_split
import open3d as o3d
import numpy as np

# Definition des PointNet-Modells
class PointNet(nn.Module):
    def __init__(self):
        super(PointNet, self).__init__()
        # Hier wird die Architektur von PointNet definiert
        # Beispiel: Fully Connected Layers, Conv1D, usw.

    def forward(self, x):
        # Hier wird die Vorwärtsdurchlauflogik für das Netzwerk definiert
        return x

# Definition des PointNet-Datasets
class CustomDataset(Dataset):
    def __init__(self, file_list, transform=None):
        self.file_list = file_list
        self.transform = transform

    def __len__(self):
        return len(self.file_list)

    def __getitem__(self, idx):
        # Lade Punktwolke und Label aus .ply und .txt-Dateien
        pointcloud_path = self.file_list[idx]
        label_path = pointcloud_path.replace('.ply', '.txt')

        pointcloud = o3d.io.read_point_cloud(pointcloud_path)
        points = np.asarray(pointcloud.points)

        # Lese Labels aus .txt-Datei
        labels = np.loadtxt(label_path, delimiter=',', skiprows=1, usecols=[3], dtype=np.int64)

        # Führe Transformation durch (falls erforderlich)
        if self.transform:
            points, labels = self.transform(points, labels)

        return {'points': points, 'labels': labels}

# Definition der Transformation (falls erforderlich)
class CustomTransform:
    def __call__(self, points, labels):
        # Hier könnten Transformationen wie Normalisierung oder Rauschen durchgeführt werden
        return points, labels

# Datenpfad und Split in Trainings- und Validierungssets
data_path = "path/to/your/data/"
file_list = ["file1.ply", "file2.ply", "file3.ply"]  # Liste der Dateien
train_files, val_files = train_test_split(file_list, test_size=0.2, random_state=42)

# Erstellen von Dataset und DataLoader
train_dataset = CustomDataset(train_files, transform=CustomTransform())
val_dataset = CustomDataset(val_files, transform=CustomTransform())

train_dataloader = DataLoader(train_dataset, batch_size=32, shuffle=True)
val_dataloader = DataLoader(val_dataset, batch_size=32, shuffle=False)

# Initialisierung des PointNet-Modells, Verlustfunktion und Optimierers
model = PointNet()
criterion = nn.CrossEntropyLoss()
optimizer = optim.Adam(model.parameters(), lr=0.001)

# Trainingsschleife
num_epochs = 10
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model.to(device)

for epoch in range(num_epochs):
    model.train()
    for batch in train_dataloader:
        points, labels = batch['points'], batch['labels']
        points, labels = torch.tensor(points, dtype=torch.float32).to(device), torch.tensor(labels, dtype=torch.long).to(device)

        optimizer.zero_grad()
        outputs = model(points)
        loss = criterion(outputs, labels)
        loss.backward()
        optimizer.step()

    # Validierungsschleife
    model.eval()
    with torch.no_grad():
        for val_batch in val_dataloader:
            val_points, val_labels = val_batch['points'], val_batch['labels']
            val_points, val_labels = torch.tensor(val_points, dtype=torch.float32).to(device), torch.tensor(val_labels, dtype=torch.long).to(device)

            val_outputs = model(val_points)
            val_loss = criterion(val_outputs, val_labels)

    print(f'Epoch {epoch+1}/{num_epochs}, Loss: {loss.item()}, Val Loss: {val_loss.item()}')
