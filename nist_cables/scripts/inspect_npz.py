import numpy as np

def inspect_npz(file_path):
    data = np.load(file_path, allow_pickle=True)
    
    print("Chiavi disponibili nel file .npz:", data.files)
    
    for key in data.files:
        array = data[key]
        print(f"\nChiave: {key}")
        print(f"Tipo di Dato: {type(array)}")
        print(f"Forma: {array.shape}")
        if isinstance(array, np.shape):
            print(f"Primi 5 valori: {array[:5]}")
            print(f"Ultimi 5 valori: {array[-5:]}")

if __name__ == "__main__":
    file_path = input("Inserisci il percorso del file .npz (relativo alla directory corrente): ")
    inspect_npz(file_path)

    