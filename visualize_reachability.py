#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

def main():
    print("Caricamento dei dati di reachability...")
    try:
        df = pd.read_csv('reachability_results.csv')
    except FileNotFoundError:
        print("Errore: file 'reachability_results.csv' non trovato.")
        return

    # Arrotondiamo Z per evitare problemi di precisione coi float e trovare i ripiani esatti
    df['Z_round'] = df['Z'].round(3)
    shelves_z = sorted(df['Z_round'].unique())
    
    print(f"Trovati {len(shelves_z)} ripiani analizzati alle quote Z: {shelves_z}")

    # Prepariamo la figura: un subplot per ogni ripiano
    fig, axes = plt.subplots(1, len(shelves_z), figsize=(6 * len(shelves_z)-10, 6))
    
    # Se c'è solo un ripiano (es. perché abbiamo saltato i primi 400 ID), axes non è una lista
    if len(shelves_z) == 1:
        axes = [axes]

    for i, z_val in enumerate(shelves_z):
        # Filtriamo i dati per il singolo ripiano
        shelf_data = df[df['Z_round'] == z_val].copy()
        
        # Arrotondiamo X e Y per raggrupparli perfettamente nella griglia
        shelf_data['X_round'] = shelf_data['X'].round(2)
        shelf_data['Y_round'] = shelf_data['Y'].round(2)
        
        # Creiamo una matrice 2D (Pivot Table) per la Heatmap
        # Indice (Righe) = Asse X (Profondità), Colonne = Asse Y (Larghezza)
        pivot_table = shelf_data.pivot_table(
            index='X_round', 
            columns='Y_round', 
            values='Reachable', 
            aggfunc='max'
        )
        
        # Custom color map: 0 = Rosso (Non raggiungibile), 1 = Verde (Raggiungibile)
        cmap = sns.color_palette(["#d73027", "#1a9850"])
        
        # Disegniamo la heatmap
        sns.heatmap(pivot_table, ax=axes[i], cmap=cmap, cbar=False, 
                    linewidths=0.5, linecolor='lightgray', square=True)
        
        # Sistemiamo le etichette e gli assi
        axes[i].set_title(f'Ripiano Z = {z_val} m\n(Raggiungibili: {shelf_data["Reachable"].sum()}/{len(shelf_data)})', fontsize=12)
        axes[i].set_xlabel('Y (Larghezza) [m]')
        axes[i].set_ylabel('X (Profondità) [m]')
        
        # Invertiamo l'asse X in modo che la parte "frontale" dello scaffale (X minore) sia in basso
        axes[i].invert_yaxis()

    plt.suptitle('TIAGo Pro: Reachability Heatmap ', fontsize=16, fontweight='bold')
    plt.tight_layout()
    
    print("Generazione grafico in corso... Chiudi la finestra per terminare.")
    plt.show()

if __name__ == '__main__':
    main()