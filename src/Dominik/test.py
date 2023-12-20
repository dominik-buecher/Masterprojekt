import os
cloudcompy_root = r"C:\Users\domin\Documents\Studium\Master\Masterprojekt-local\CloudComPy310"
python_path1 = os.path.join(cloudcompy_root, 'CloudCompare')
python_path2 = os.path.join(cloudcompy_root, 'doc', 'PythonAPI_test')

# Setze die Umgebungsvariablen
os.environ['CLOUDCOMPY_ROOT'] = cloudcompy_root
os.environ['PYTHONPATH'] = f"{python_path1};{python_path2};{os.environ.get('PYTHONPATH', '')}"
os.environ['PATH'] = f"{python_path1};{os.path.join(cloudcompy_root, 'ccViewer')};{os.environ['PATH']}"
os.environ['PATH'] = f"{os.path.join(cloudcompy_root, 'CloudCompare', 'plugins')};{os.environ['PATH']}"
os.system("python checkenv.py")