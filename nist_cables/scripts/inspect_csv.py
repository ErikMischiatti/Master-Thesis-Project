import pandas as pd
import glob
import os

def analyze_csv_files(csv_dir):
    csv_files = glob.glob(os.path.join(csv_dir, "*.csv"))
    
    for file in csv_files:
        df = pd.read_csv(file)
        print(f"\nAnalisi del file: {file}")
        print(f"Colonne disponibili: {df.columns}")
        print(f"Numero di righe: {len(df)}")
        if 'Time' in df.columns:
            print(f"Primi 5 valori della colonna 'Time':\n{df['Time'].head()}")
            print(f"Ultimi 5 valori della colonna 'Time':\n{df['Time'].tail()}")
        else:
            print("La colonna 'Time' non è presente in questo file.")

if __name__ == "__main__":
    csv_dir = input("Inserisci il percorso della directory contenente i file CSV: ")
    analyze_csv_files(csv_dir)
